#!/usr/bin/env python3
"""
MR Summary Script
在 GitLab MR 中获取 Diff，调用内网 LLM API 生成功能变更总结，
并以覆盖式评论发回 MR。

依赖的环境变量:
  CI_MERGE_REQUEST_IID  - GitLab CI 自动注入
  CI_PROJECT_ID         - GitLab CI 自动注入
  CI_API_V4_URL         - GitLab CI 自动注入
  GITLAB_API_TOKEN      - 用户配置
  LLM_AUTH_TOKEN        - 用户配置
  LLM_API_URL           - (可选) 默认 baitong-ai
  LLM_MODEL             - (可选) 默认 qwen3-coder-flash
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
log("===== MR Summary Starting =====")

CI_MERGE_REQUEST_IID = require_env("CI_MERGE_REQUEST_IID")
CI_PROJECT_ID = require_env("CI_PROJECT_ID")
CI_API_V4_URL = require_env("CI_API_V4_URL")
GITLAB_API_TOKEN = require_env("GITLAB_API_TOKEN")
LLM_AUTH_TOKEN = require_env("LLM_AUTH_TOKEN")

log(f"MR IID:    {CI_MERGE_REQUEST_IID}")
log(f"Project:   {CI_PROJECT_ID}")
log(f"Model:     {LLM_MODEL}")


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


def gitlab_api_delete(path: str):
    url = f"{CI_API_V4_URL}/projects/{CI_PROJECT_ID}/merge_requests/{CI_MERGE_REQUEST_IID}/{path}"
    req = urllib.request.Request(url, method="DELETE")
    req.add_header("PRIVATE-TOKEN", GITLAB_API_TOKEN)
    try:
        urllib.request.urlopen(req, timeout=30)
    except Exception:
        pass


# ---------- 获取 MR Diff ----------
log("正在获取 MR Diff ...")
changes_data = gitlab_api_get("changes")
changes = changes_data.get("changes", [])
if not changes:
    log("无文件变更，跳过总结")
    sys.exit(0)


# ---------- 提取文件变更统计 ----------
file_lines = []
total_new = total_modified = total_deleted = 0

for ch in changes:
    path = ch.get("new_path", ch.get("old_path", "unknown"))
    new_file = ch.get("new_file", False)
    deleted_file = ch.get("deleted_file", False)
    renamed_file = ch.get("renamed_file", False)
    diff = ch.get("diff", "")

    if new_file:
        status = "➕ 新增"
        total_new += 1
    elif deleted_file:
        status = "❌ 删除"
        total_deleted += 1
    elif renamed_file:
        status = "✏️  重命名"
    else:
        status = "📝 修改"
        total_modified += 1

    lines_added = sum(1 for line in diff.split("\n") if line.startswith("+") and not line.startswith("+++"))
    lines_removed = sum(1 for line in diff.split("\n") if line.startswith("-") and not line.startswith("---"))
    file_lines.append(f"{status} {path}  (+{lines_added}/-{lines_removed})")

file_list = "\n".join(file_lines)
stats = f"新增:{total_new} 修改:{total_modified} 删除:{total_deleted}"
log(f"变更统计: {stats}")

# ---------- 提取并裁剪 Diff ----------
diff_parts = []
total_len = 0
for ch in changes:
    path = ch.get("new_path", ch.get("old_path", "unknown"))
    diff = ch.get("diff", "")
    header = f"\n=== {path} ===\n"
    remaining = MAX_DIFF_CHARS - total_len - len(header)
    if remaining <= 0:
        break
    if len(diff) > remaining:
        diff_parts.append(header + diff[:remaining] + "\n...(truncated)")
        total_len += remaining + len(header)
    else:
        diff_parts.append(header + diff)
        total_len += len(diff) + len(header)

diff_text = "".join(diff_parts)
if not diff_text.strip():
    log("Diff 为空，跳过总结")
    sys.exit(0)


# ---------- 调用 LLM API 生成总结 ----------
log(f"正在调用 LLM API ({LLM_MODEL}) 生成总结 ...")

SYSTEM_PROMPT = """你是一位资深的项目助理，负责为 Git Merge Request 撰写功能变更总结。

## 输出要求
请用中文输出，格式为 Markdown：

### 📋 变更概览
- 本次 MR 涉及的变更统计

### 📁 修改文件清单
- 逐行列出变更的文件及增删行数

### 🎯 功能总结
- 用 3-5 条简洁要点总结本次提交的核心功能变更
- 每条以 "- " 开头，一句话说明
- 关注业务功能变化，而非代码实现细节
- 如果涉及多个功能模块，按模块分类说明

### ⚠️ 注意事项
- 如有破坏性变更（接口变更、配置变更、数据库变更），请在此注明
- 如无则写"无"

请先自行推导变更内容，再按以上格式输出。"""

USER_PROMPT = f"""以下是 MR 的代码变更 Diff：

```diff
{diff_text}
```

文件变更清单：
{file_list}

统计信息：{stats}

请按格式输出功能总结。"""

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
    warn(f"LLM API 调用失败，跳过总结: {last_error}")
    sys.exit(0)

# ---------- 解析 LLM 回复 ----------
try:
    summary_content = llm_response["choices"][0]["message"]["content"] or ""
except (KeyError, IndexError, TypeError) as e:
    warn(f"解析 LLM 响应失败: {e}")
    sys.exit(0)

log("总结生成完成")


# ---------- 检查并删除已有总结评论（覆盖式） ----------
log("正在检查 MR 上是否已有总结评论 ...")
MARKER = "<!-- ai-summary -->"

try:
    notes = gitlab_api_get("notes?sort=desc&per_page=100")
    ai_note_ids = [n["id"] for n in notes if MARKER in n.get("body", "")]
    ai_note_ids.sort()
    for note_id in ai_note_ids:
        gitlab_api_delete(f"notes/{note_id}")
        log(f"已删除旧总结 ID={note_id}")
except Exception as e:
    warn(f"处理旧总结时出现异常: {e}")


# ---------- 发送总结到 MR 评论 ----------
log("正在发送总结评论到 MR ...")

if summary_content.strip():
    body = f"""## 📋 MR 功能变更总结

**评审模型**: `{LLM_MODEL}`

---

{summary_content}

---
{MARKER}"""
else:
    body = f"""## 📋 MR 功能变更总结

**评审模型**: `{LLM_MODEL}`

---

总结未返回具体内容。

---
{MARKER}"""

gitlab_api_post("notes", {"body": body})
log(f"✅ MR 功能总结完成，已发布到 MR !{CI_MERGE_REQUEST_IID}")
