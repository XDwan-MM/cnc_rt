#include "json_read.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cJSON.h"

// 从文件读取内容到字符串
static char* read_file_to_string(const char *filepath) {
    FILE *file = fopen(filepath, "rb");
    if (!file) {
        fprintf(stderr, "Could not open file: %s\n", filepath);
        return NULL;
    }

    fseek(file, 0, SEEK_END);
    long length = ftell(file);
    fseek(file, 0, SEEK_SET);

    char *content = (char *)malloc(length + 1);
    if (!content) {
        fprintf(stderr, "Memory allocation failed.\n");
        fclose(file);
        return NULL;
    }

    fread(content, 1, length, file);
    content[length] = '\0';
    fclose(file);

    return content;
}

/// @brief 读取机床参数json文件
/// @param filepath 
/// @param params 
/// @return 
/// @version v1.0
/// @author 许晓婷
/// @date 2025/10/16
int parse_param_file(const char *filepath, PARAMS *params) {
    char *json_string = read_file_to_string(filepath);
    if (!json_string) {
        return -1;
    }

    cJSON *root = cJSON_Parse(json_string);
    free(json_string);
    if (!root) {
        fprintf(stderr, "JSON parse error: [%s]\n", cJSON_GetErrorPtr());
        return -1;
    }

    cJSON *cycle_time = cJSON_GetObjectItem(root, "cycle_time");
    cJSON *feed_vel = cJSON_GetObjectItem(root, "feed_vel");
    cJSON *feed_acc = cJSON_GetObjectItem(root, "feed_acc");
    cJSON *feed_jerk = cJSON_GetObjectItem(root, "feed_jerk");

    if (cJSON_IsNumber(feed_vel) && cJSON_IsNumber(feed_acc) && cJSON_IsNumber(feed_jerk)) {
        params->cycle_time= cycle_time->valuedouble;
        params->feed_vel = feed_vel->valuedouble;
        params->feed_acc = feed_acc->valuedouble;
        params->feed_jerk = feed_jerk->valuedouble;
    } else {
        cJSON_Delete(root);
        return -1;
    }

    cJSON_Delete(root);
    return 0;
}

static INTERP_TYPE string_to_segment_type(const char *type_str) {
    if (strcmp(type_str, "RAPID") == 0) {
        return RAPID;
    } else if (strcmp(type_str, "LINE") == 0) {
        return LINE;
    } else if (strcmp(type_str, "CIRCLE_CW") == 0) {
        return CIRCLE_CW;
    }else if (strcmp(type_str, "CIRCLE_CCW") == 0) {
        return CIRCLE_CCW;
    }
    return UNKNOWN;
}

static int parse_position(cJSON *pos_json, POS *pos) {
    cJSON *x = cJSON_GetObjectItem(pos_json, "x");
    cJSON *y = cJSON_GetObjectItem(pos_json, "y");
    cJSON *z = cJSON_GetObjectItem(pos_json, "z");

    if (cJSON_IsString(x) && cJSON_IsString(y) && cJSON_IsString(z)) {
        pos->x = atof(x->valuestring);
        pos->y = atof(y->valuestring);
        pos->z = atof(z->valuestring);
        return 0;
    }
    return -1;
}

/// @brief 读取插补队列json文件
/// @param filepath 
/// @param pl_queue 
/// @return 
/// @version v1.0
/// @author 许晓婷
/// @date 2025/10/16
int parse_tp_file(const char *filepath, PLAN_QUEUE* pl_queue) {
    char *json_string = read_file_to_string(filepath);
    if (!json_string) {
        return -1;
    }

    cJSON *root = cJSON_Parse(json_string);
    free(json_string);
    if (!root) {
        fprintf(stderr, "JSON parse error: [%s]\n", cJSON_GetErrorPtr());
        return -1;
    }

    if (!cJSON_IsArray(root)) {
        cJSON_Delete(root);
        return -1;
    }

    int num_segments = cJSON_GetArraySize(root);
    PLAN_BLOCK *tc_array = (PLAN_BLOCK*)malloc(num_segments * sizeof(PLAN_BLOCK));
    pl_queue->pl_block = tc_array;
    if (!tc_array) {
        cJSON_Delete(root);
        return -1;
    }

    for (int i = 0; i < num_segments; i++) {
        cJSON *segment_json = cJSON_GetArrayItem(root, i);
        PLAN_BLOCK *current_segment = &((tc_array)[i]);

        cJSON *type = cJSON_GetObjectItem(segment_json, "type");
        cJSON *length = cJSON_GetObjectItem(segment_json, "length");
        cJSON *start_vel = cJSON_GetObjectItem(segment_json, "start_vel");
        cJSON *end_vel = cJSON_GetObjectItem(segment_json, "end_vel");
        cJSON *max_vel = cJSON_GetObjectItem(segment_json, "max_vel");
        cJSON *start_pos_json = cJSON_GetObjectItem(segment_json, "start_pos"); 
        cJSON *end_pos_json = cJSON_GetObjectItem(segment_json, "end_pos");

        if (cJSON_IsString(type) && cJSON_IsString(length) && cJSON_IsString(start_vel) && cJSON_IsString(end_vel) && cJSON_IsString(max_vel)) {
            current_segment->type = string_to_segment_type(type->valuestring);
            current_segment->length = atof(length->valuestring);
            current_segment->start_vel = atof(start_vel->valuestring);
            current_segment->end_vel = atof(end_vel->valuestring);
            current_segment->max_vel = atof(max_vel->valuestring);
        }
        else{
            cJSON_Delete(root);
            return -1;
        }

        if (parse_position(start_pos_json, &current_segment->start_pos) != 0) {
             fprintf(stderr, "Failed to parse start_pos for segment %d\n", i);
        }
        if (parse_position(end_pos_json, &current_segment->end_pos) != 0) {
            fprintf(stderr, "Failed to parse end_pos for segment %d\n", i);
        }
    }

    cJSON_Delete(root);
    return num_segments;
}

/// @brief 读取解析队列json文件
/// @param filepath 
/// @param dcd_array 
/// @return 
/// @version v1.0
/// @author 许晓婷
/// @date 2025/10/16
int parse_dcd_file(const char *filepath, GCODE_QUEUE *gcode_queue){
    char *json_string = read_file_to_string(filepath);
    if (!json_string) {
        return -1;
    }

    cJSON *root = cJSON_Parse(json_string);
    free(json_string);
    if (!root) {
        fprintf(stderr, "JSON parse error: [%s]\n", cJSON_GetErrorPtr());
        return -1;
    }

    if (!cJSON_IsArray(root)) {
        cJSON_Delete(root);
        return -1;
    }

    int num_segments = cJSON_GetArraySize(root);
    if (!num_segments) {
        cJSON_Delete(root);
        return -1;
    }

    GCODE_BLOCK *current_block = (GCODE_BLOCK*)malloc(sizeof(GCODE_BLOCK));
    for (int i = 0; i < num_segments; i++) {
        cJSON *segment_json = cJSON_GetArrayItem(root, i);

        cJSON *type = cJSON_GetObjectItem(segment_json, "type");
        cJSON *feed_json = cJSON_GetObjectItem(segment_json, "feed_vel");
        cJSON *pos_json = cJSON_GetObjectItem(segment_json, "pos");

        if (cJSON_IsString(type)) {
            current_block->type = string_to_segment_type(type->valuestring);
        }else{
            cJSON_Delete(root);
            return -1;
        }
        if (parse_position(pos_json, &current_block->pos) != 0) {
            fprintf(stderr, "Failed to parse pos for segment %d\n", i);
        }
        if (cJSON_IsNumber(feed_json)){
            current_block->feed_vel = feed_json->valuedouble;
        }
        gcode_queue_push(gcode_queue, current_block);
    }

    cJSON_Delete(root);
    return num_segments;
}