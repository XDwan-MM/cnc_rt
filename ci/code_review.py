#!/usr/bin/env python3
"""
AI Code Review Script
在 GitLab MR 中获取 Diff，调用内网 LLM API 进行代码评审，
并以覆盖式评论发回 MR。

依赖的环境变量:
  CI_MERGE_REQUEST_IID  - GitLab CI 自动注入
  CI_PROJECT_ID         - GitLab CI 自动注入
  CI_API_V4_URL         - GitLab CI 自动注入
  GITLAB_API_TOKEN      - 用户配置
  LLM_AUTH_TOKEN        - 用户配置
  LLM_API_URL           - (可选) 默认 baitong-ai
  LLM_MODEL             - (可选) 默认 qwen3-coder-plus
  MAX_DIFF_CHARS        - (可选) 默认 8000
"""
import json
import os
import sys
import tempfile
import time
import urllib.request
import urllib.error


# ---------- 配置 ----------
LLM_API_URL = os.environ.get(
    "LLM_API_URL",
    "https://baitong-ai.gree.com/openapi/llm/chat/completions",
)
LLM_MODEL = os.environ.get("LLM_MODEL", "Qwen-Flash")
MAX_DIFF_CHARS = int(os.environ.get("MAX_DIFF_CHARS", "8000"))


def log(msg: str):
    print(f"[INFO] {msg}")


def warn(msg: str):
    print(f"[WARN] {msg}", file=sys.stderr)


def error(msg: str):
    print(f"[ERROR] {msg}", file=sys.stderr)


def require_env(name: str) -> str:
    val = os.environ.get(name)
    if not val:
        error(f"{name} 未设置")
        sys.exit(1)
    return val.strip()


# ---------- 前置检查 ----------
log("===== AI Code Review Starting =====")

CI_MERGE_REQUEST_IID = require_env("CI_MERGE_REQUEST_IID")
CI_PROJECT_ID = require_env("CI_PROJECT_ID")
CI_API_V4_URL = require_env("CI_API_V4_URL")
GITLAB_API_TOKEN = require_env("GITLAB_API_TOKEN")
LLM_AUTH_TOKEN = require_env("LLM_AUTH_TOKEN")

log(f"MR IID:    {CI_MERGE_REQUEST_IID}")
log(f"Project:   {CI_PROJECT_ID}")
log(f"Model:     {LLM_MODEL}")


# ---------- 临时目录 ----------
TMPDIR = tempfile.mkdtemp(prefix="code_review_")
log(f"Temp dir:  {TMPDIR}")


# ---------- GitLab API 请求封装 ----------
def gitlab_api_get(path: str) -> dict:
    url = f"{CI_API_V4_URL}/projects/{CI_PROJECT_ID}/merge_requests/{CI_MERGE_REQUEST_IID}/{path}"
    req = urllib.request.Request(url)
    req.add_header("PRIVATE-TOKEN", GITLAB_API_TOKEN)
    try:
        resp = urllib.request.urlopen(req, timeout=30)
        return json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        error(f"GitLab API GET {path} 失败: HTTP {e.code} {e.reason}")
        sys.exit(1)
    except Exception as e:
        error(f"GitLab API GET {path} 失败: {e}")
        sys.exit(1)


def gitlab_api_delete(path: str):
    url = f"{CI_API_V4_URL}/projects/{CI_PROJECT_ID}/merge_requests/{CI_MERGE_REQUEST_IID}/{path}"
    req = urllib.request.Request(url, method="DELETE")
    req.add_header("PRIVATE-TOKEN", GITLAB_API_TOKEN)
    try:
        urllib.request.urlopen(req, timeout=30)
    except Exception:
        pass  # 删除失败不打紧


def gitlab_api_post(path: str, data: dict):
    url = f"{CI_API_V4_URL}/projects/{CI_PROJECT_ID}/merge_requests/{CI_MERGE_REQUEST_IID}/{path}"
    payload = json.dumps(data).encode("utf-8")
    req = urllib.request.Request(url, data=payload, method="POST")
    req.add_header("PRIVATE-TOKEN", GITLAB_API_TOKEN)
    req.add_header("Content-Type", "application/json")
    try:
        urllib.request.urlopen(req, timeout=30)
    except urllib.error.HTTPError as e:
        body = e.read().decode("utf-8", errors="replace")
        warn(f"GitLab API POST {path} 失败: HTTP {e.code} {body[:200]}")
    except Exception as e:
        warn(f"GitLab API POST {path} 失败: {e}")


# ---------- 获取 MR Diff ----------
log("正在获取 MR Diff ...")
changes_data = gitlab_api_get("changes")
changes = changes_data.get("changes", [])
if not changes:
    log("Diff 为空，跳过评审")
    sys.exit(0)


# ---------- 提取并裁剪 Diff ----------
diff_parts = []
total_len = 0

for ch in changes:
    path = ch.get("new_path", ch.get("old_path", "unknown"))
    diff = ch.get("diff", "")
    header = f"\n=== {path} ===\n"
    remaining = MAX_DIFF_CHARS - total_len - len(header)
    if remaining <= 0:
        diff_parts.append(header + "(diff truncated due to size limit)\n")
        continue
    if len(diff) > remaining:
        diff_parts.append(header + diff[:remaining] + "\n...(truncated)")
        total_len += remaining + len(header)
    else:
        diff_parts.append(header + diff)
        total_len += len(diff) + len(header)

diff_text = "".join(diff_parts)
if not diff_text.strip():
    log("Diff 为空，跳过评审")
    sys.exit(0)

log(f"Diff 大小: {len(diff_text)} 字符")


# ---------- 调用 LLM API ----------
log(f"正在调用 LLM API ({LLM_MODEL}) ...")

SYSTEM_PROMPT = """你是一位资深的 C++/Qt 代码评审专家、OCC三维几何库专家。本次评审**1、检测编码的规范问题 2、代码逻辑合理性**，不要过度分析，不要报复杂逻辑问题。

## 必检项（只有这 4 类）

### 1. 命名规范问题
| 文件 | 行号 | 问题描述 |
|------|------|----------|
| 文件路径 | 行号 | 具体问题说明 |

检查规则：
- 类名应为 `大驼峰`（`class MyClass`），不能是 `my_class`、`myClass`
- 函数/方法名应为 `大驼峰`（`void MyFunction()`），不能是 `myFunction`、`my_function`
- 类成员变量名应为 `大驼峰` （`int MyVar` ），不能是 `int myVar` 或`my_var`
- 局部变量名应为 `小驼峰` (`int myVar`)
- 宏/常量/枚举值应为 `全大写_下划线`（`#define MAX_SIZE 100`）
- Qt 信号/槽名称应为 `大驼峰`或`下划线`
- 文件名应为 `大驼峰` 风格（`MyWidget.cpp`）

### 2. 头文件规范问题
| 文件 | 行号 | 问题描述 |
|------|------|----------|
| 文件路径 | 行号 | 具体问题说明 |

检查规则：
- `.h` 文件缺少 `#pragma once` 或头文件保护宏
- Qt 类声明中缺少 `Q_OBJECT` 宏（使用了 signals/slots 但未声明）

### 3. 明显的安全问题
| 文件 | 行号 | 问题描述 |
|------|------|----------|
| 文件路径 | 行号 | 具体问题说明 |

检查规则：
- 使用 `sprintf`、`strcpy`、`strcat` 等不安全的 C 函数（应使用 `snprintf`、`QString`）
- `new` 后没有对应的 `delete`（在构造函数/初始化中分配未在析构函数中释放）
- C++容器使用规范等问题

### 4. 硬编码问题
| 文件 | 行号 | 问题描述 |
|------|------|----------|
| 文件路径 | 行号 | 具体问题说明 |

检查规则：
- 代码中直接硬编码了 IP 地址、密码、Token、密钥等敏感信息
- 魔法数字（magic number）直接写在代码中而未定义常量（仅限明显的数字，如 `new int[100]` 中的 `100`）

## 输出规则
- **只输出上述 4 类中确定存在的问题**，不确定的不输出
- 每类表格如果无问题则省略
- 不需要输出故障类型统计
- 如果以上 4 类都未发现问题，输出："✅ 本次变更未发现命名规范、头文件、安全或硬编码问题"

## 核心约束
- 只检查 diff 中**新增和修改的代码行**
- 每条问题必须附上**真实的文件和行号**
- **不要捏造行号**，不确定行号就不输出该条
- 指出代码逻辑不合理的地方
- 分析OCC库使用不合理之处
- 除非上述 4 类明确规定，否则**不要检查其他问题**（如性能、注释质量等）"""

USER_PROMPT = f"""以下是 C++/Qt 项目的 MR 代码变更 Diff，请检查简单规范问题：

```diff
{diff_text}
```

**注意**：只检查命名规范、头文件规范、安全问题和硬编码这 4 类，不要分析代码逻辑。"""

payload = json.dumps(
    {
        "model": LLM_MODEL,
        "messages": [
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": USER_PROMPT},
        ],
        "stream": False,
        "enable_thinking": False,
    }
).encode("utf-8")

# 带重试的 API 调用
llm_response = None
last_error = None

for attempt in range(3):
    try:
        req = urllib.request.Request(LLM_API_URL, data=payload, method="POST")
        req.add_header("Authorization", f"Bearer {LLM_AUTH_TOKEN}")
        req.add_header("Content-Type", "application/json; charset=utf-8")
        resp = urllib.request.urlopen(req, timeout=180)
        # 验证 HTTP 状态码
        if resp.status != 200:
            warn(f"LLM API 返回非 200 状态码: {resp.status}")
            last_error = f"HTTP {resp.status}"
            continue
        llm_response = json.loads(resp.read().decode("utf-8"))
        last_error = None
        break
    except (urllib.error.URLError, urllib.error.HTTPError) as e:
        last_error = str(e)
        if attempt < 2:
            wait = 2**attempt
            log(f"LLM API 调用失败，{wait}s 后重试 ({attempt+1}/3): {e}")
            time.sleep(wait)
    except json.JSONDecodeError as e:
        last_error = f"JSON 解析失败: {e}"
        break

if last_error or llm_response is None:
    warn(f"LLM API 调用失败，跳过评审: {last_error}")
    sys.exit(0)

# ---------- 解析 LLM 回复 ----------
try:
    review_content = llm_response["choices"][0]["message"]["content"] or ""
except (KeyError, IndexError, TypeError) as e:
    warn(f"解析 LLM 响应失败: {e}")
    sys.exit(0)

log("评审完成")


# ---------- 发送评审结果到 MR 评论（增量追加） ----------
log("正在发送评审评论到 MR ...")
MARKER = "<!-- ai-code-review -->"

if review_content.strip():
    body = f"""## 🤖 AI 代码评审报告

**评审模型**: `{LLM_MODEL}`
**评审时间**: `{time.strftime('%Y-%m-%d %H:%M:%S')}`

---

{review_content}

---
{MARKER}"""
else:
    body = f"""## 🤖 AI 代码评审报告

**评审模型**: `{LLM_MODEL}`
**评审时间**: `{time.strftime('%Y-%m-%d %H:%M:%S')}`

---

评审未返回具体内容，可能 Diff 中无需要关注的问题。

---
{MARKER}"""

gitlab_api_post("notes", {"body": body})
log(f"✅ AI 代码评审完成，评论已发布到 MR !{CI_MERGE_REQUEST_IID}")
