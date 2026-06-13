#ifndef _FILTER_H_          // 如果没有定义 _FILTER_H_ 这个宏
#define _FILTER_H_          // 那么就定义它（这是防止头文件被重复包含的标准写法）

#include "include.h"        // 包含项目的主头文件，里面有常用的类型定义

/* 滤波器的配置参数 */
#define MAX_FILTER_SIZE     10     // 定义最大窗口尺寸为10，就是最多存储10个历史数据
#define STABILITY_THRESHOLD 5      // 稳定性阈值设为5，当数据波动小于5时认为稳定

/* 移动平均滤波器的结构体定义 */
typedef struct {                    // 定义一个结构体类型
    uint8_t buffer[MAX_FILTER_SIZE];  // 数据缓冲区，用于存储历史数据
    uint8_t buffer_size;              // 实际的窗口大小（比如3、5、10等）
    uint8_t current_index;            // 当前索引，指示下一个数据放在哪个位置
    uint16_t sum;                     // 当前缓冲区中所有数据的总和
    uint8_t is_initialized;           // 初始化标志，1表示已初始化，0表示未初始化
} MovingAverageFilter;                // 给这个结构体类型取个名字

/* 一阶低通滤波器的结构体定义 */
typedef struct {                    // 定义另一个结构体类型
    float alpha;                    // 滤波系数α，取值范围0.0~1.0
    float output;                   // 当前滤波器的输出值
    uint8_t is_initialized;         // 初始化标志
} LowPassFilter;                    // 给这个结构体类型取个名字

/* 下面是函数声明，告诉编译器这些函数的存在 */
/* 移动平均滤波相关函数 */
void MovingAverage_Init(MovingAverageFilter* filter, uint8_t buffer_size);
uint8_t MovingAverage_Update(MovingAverageFilter* filter, uint8_t new_value);
void MovingAverage_Reset(MovingAverageFilter* filter);
uint8_t MovingAverage_IsStable(MovingAverageFilter* filter);

/* 一阶低通滤波相关函数 */
void LowPassFilter_Init(LowPassFilter* filter, float alpha, uint8_t initial_value);
float LowPassFilter_UpdateFloat(LowPassFilter* filter, uint8_t new_value);
uint8_t LowPassFilter_UpdateInt(LowPassFilter* filter, uint8_t new_value);
void LowPassFilter_Reset(LowPassFilter* filter, uint8_t new_value);
void LowPassFilter_AdjustAlpha(LowPassFilter* filter, float new_alpha);

#endif /* _FILTER_H_ */     // 结束ifndef的匹配
