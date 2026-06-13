#include "include.h"        // 包含项目的主头文件，里面有常用的类型定义

/**
 * 初始化移动平均滤波器
 * 
 * 想象一下：你有一个盒子（buffer），可以放很多小球（数据）
 * 这个函数就是准备盒子，告诉你要放多少个小球
 */
void MovingAverage_Init(MovingAverageFilter* filter, uint8_t buffer_size)
{
    uint8_t i;  // 定义一个循环变量i，用于循环
    
    /* 第一步：参数检查（非常重要！） */
    if(buffer_size == 0) buffer_size = 1;  // 如果有人传0，就改成1（至少要存1个数）
    if(buffer_size > MAX_FILTER_SIZE)      // 如果超过最大容量
        buffer_size = MAX_FILTER_SIZE;     // 就限制为最大容量
    
    /* 第二步：初始化结构体的各个成员 */
    filter->buffer_size = buffer_size;     // 设置窗口大小
    filter->current_index = 0;             // 从第一个位置开始放数据
    filter->sum = 0;                       // 总和初始化为0
    filter->is_initialized = 1;            // 标记为已初始化
    
    /* 第三步：清空缓冲区（把盒子里的所有位置都设为0） */
    for(i = 0; i < buffer_size; i++)       // 循环buffer_size次
    {
        filter->buffer[i] = 0;             // 把每个位置都设为0
    }
    
    /* 调试信息（可以用串口输出，调试时打开） */
    // printf("移动平均滤波初始化完成，窗口大小=%d\r\n", buffer_size);
}

/**
 * 更新移动平均滤波器（核心函数）
 * 
 * 工作原理：就像排队一样，新来的数据排到队尾，最老的数据离开队伍
 * 然后计算队伍里所有人的平均值
 */
uint8_t MovingAverage_Update(MovingAverageFilter* filter, uint8_t new_value)
{
    /* 第一步：检查滤波器是否初始化了 */
    if(!filter->is_initialized)            // 如果没初始化（!表示取反，0变成1）
    {
        return new_value;                  // 直接返回原始值，不做任何处理
    }
    
    /* 第二步：更新缓冲区（最关键的部分） */
    /* 假设我们有3个位置：[0]、[1]、[2]
       第一次：current_index=0，新数据放在[0]，sum=0+新值
       第二次：current_index=1，新数据放在[1]，sum=上次sum+新值
       第三次：current_index=2，新数据放在[2]，sum=上次sum+新值
       第四次：current_index=0，新数据放在[0]，sum=上次sum-原来[0]的值+新值
    */
    filter->sum -= filter->buffer[filter->current_index];  // 从总和中减去要被替换的旧值
    filter->buffer[filter->current_index] = new_value;     // 把新值放入当前位置
    filter->sum += new_value;                              // 把新值加到总和中
    
    /* 第三步：更新索引（实现环形缓冲区） */
    filter->current_index++;                     // 索引加1，指向下一个位置
    if(filter->current_index >= filter->buffer_size)  // 如果超出范围
    {
        filter->current_index = 0;               // 回到开头（实现循环）
    }
    
    /* 第四步：计算并返回平均值 */
    return (uint8_t)(filter->sum / filter->buffer_size);  // 总和除以个数得到平均值
}

/**
 * 检查滤波器是否稳定
 * 
 * 原理：比较缓冲区里相邻数据的差值
 * 如果所有相邻数据的差值都很小，说明数据稳定
 */
uint8_t MovingAverage_IsStable(MovingAverageFilter* filter)
{
    uint8_t i;                     // 循环变量
    uint8_t max_diff = 0;          // 最大差值，初始化为0
    
    if(!filter->is_initialized)    // 如果没初始化
        return 0;                  // 返回0（不稳定）
    
    /* 计算缓冲区中相邻数据的最大差值 */
    for(i = 0; i < filter->buffer_size - 1; i++)  // 循环buffer_size-1次
    {
        /* 计算第i个和第i+1个数据的绝对差值 */
        uint8_t diff = abs((int)filter->buffer[i] - (int)filter->buffer[i + 1]);
        
        if(diff > max_diff)        // 如果这个差值比当前最大值还大
            max_diff = diff;       // 更新最大值
    }
    
    /* 判断是否稳定 */
    if(max_diff < STABILITY_THRESHOLD)  // 如果最大差值小于阈值（比如5）
        return 1;                       // 返回1（稳定）
    else                                // 否则
        return 0;                       // 返回0（不稳定）
}

/**
 * 初始化一阶低通滤波器
 * 
 * 一阶低通滤波就像是一个"惯性系统"：
 * 输出不能突变，而是慢慢地跟随输入变化
 * α（alpha）控制跟随速度：α小=慢跟随，α大=快跟随
 */
void LowPassFilter_Init(LowPassFilter* filter, float alpha, uint8_t initial_value)
{
    /* 第一步：参数检查（确保alpha在合理范围内） */
    if(alpha < 0.0f) alpha = 0.0f;    // 如果小于0，设为0
    if(alpha > 1.0f) alpha = 1.0f;    // 如果大于1，设为1
    
    /* 第二步：初始化结构体成员 */
    filter->alpha = alpha;                     // 设置滤波系数
    filter->output = (float)initial_value;     // 设置初始输出值
    filter->is_initialized = 1;                // 标记为已初始化
}

/**
 * 一阶低通滤波更新（浮点版本）
 * 
 * 核心公式：y[n] = α × x[n] + (1-α) × y[n-1]
 * 其中：
 *   y[n] = 当前输出（我们要求的值）
 *   x[n] = 当前输入（新采集的数据）
 *   y[n-1] = 上一次的输出
 *   α = 滤波系数
 *   
 * 这个公式的意思是：
 *   新输出 = α × 新输入 + (1-α) × 旧输出
 *   
 * 当α=1时：新输出完全等于新输入（无滤波）
 * 当α=0时：新输出完全等于旧输出（输入被完全忽略）
 * 当α=0.3时：新输出的70%来自旧输出，30%来自新输入
 */
float LowPassFilter_UpdateFloat(LowPassFilter* filter, uint8_t new_value)
{
    /* 检查滤波器是否初始化 */
    if(!filter->is_initialized)                // 如果没初始化
    {
        return (float)new_value;               // 直接返回新值（转为浮点数）
    }
    
    /* 一阶低通滤波计算（核心！） */
    float input = (float)new_value;            // 把新值转为浮点数
    /* 计算公式：新输出 = α × 新输入 + (1-α) × 旧输出 */
    filter->output = filter->alpha * input + (1.0f - filter->alpha) * filter->output;
    
    return filter->output;                     // 返回滤波后的值
}

/**
 * 一阶低通滤波更新（整数版本）
 * 
 * 为什么要有整数版本？
 * 因为有些单片机没有浮点运算单元（FPU），浮点运算很慢
 * 整数版本用定点数运算，速度快很多
 */
uint8_t LowPassFilter_UpdateInt(LowPassFilter* filter, uint8_t new_value)
{
    /* 检查滤波器是否初始化 */
    if(!filter->is_initialized)
    {
        return new_value;                     // 直接返回原始值
    }
    
    /* 定点数运算（Q8格式：用256表示1.0） */
    /* 把浮点系数α转换为定点数（乘以256） */
    uint16_t alpha_q8 = (uint16_t)(filter->alpha * 256.0f);
    
    /* 计算公式的定点数版本：
       原来的：output = α * input + (1-α) * output
       转换为：output = (α*256*input + (256-α*256)*output) / 256
    */
    uint32_t temp = (alpha_q8 * new_value) +    // α * input（定点数乘法）
                   ((256 - alpha_q8) * (uint16_t)filter->output);  // (1-α) * output
    
    /* 除以256（相当于右移8位） */
    filter->output = (float)(temp >> 8);       // 右移8位相当于除以256
    
    /* 返回整数结果 */
    return (uint8_t)filter->output;
}

