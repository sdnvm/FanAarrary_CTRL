#include "protocol.h"
#include <string.h>
#include "control.h"
#include "bsp_pid.h"
#include "tim.h"
#include "sin_control.h"
#include "OLED.h"

struct prot_frame_parser_t
{
    uint8_t *recv_ptr;
    uint16_t r_oft;
    uint16_t w_oft;
    uint16_t frame_len;
    uint16_t found_frame_head;
};

static struct prot_frame_parser_t parser;

static uint8_t recv_buf[PROT_FRAME_LEN_RECV];

/**
  * @brief 计算校验和
  * @param ptr：需要计算的数据
  * @param len：需要计算的长度
  * @retval 校验和
  */
uint8_t check_sum(uint8_t init, uint8_t *ptr, uint8_t len )
{
  uint8_t sum = init;
  
  while(len--)
  {
    sum += *ptr;
    ptr++;
  }
  
  return sum;
}

/**
 * @brief   得到帧类型（帧命令）
 * @param   *frame:  数据帧
 * @param   head_oft: 帧头的偏移位置
 * @return  帧长度.
 */
static uint8_t get_frame_type(uint8_t *frame, uint16_t head_oft)
{
    return (frame[(head_oft + CMD_INDEX_VAL) % PROT_FRAME_LEN_RECV] & 0xFF);
}

/**
 * @brief   得到帧长度
 * @param   *buf:  数据缓冲区.
 * @param   head_oft: 帧头的偏移位置
 * @return  帧长度.
 */
static uint16_t get_frame_len(uint8_t *frame, uint16_t head_oft)
{
    return ((frame[(head_oft + LEN_INDEX_VAL + 0) % PROT_FRAME_LEN_RECV] <<  0) |
            (frame[(head_oft + LEN_INDEX_VAL + 1) % PROT_FRAME_LEN_RECV] <<  8) |
            (frame[(head_oft + LEN_INDEX_VAL + 2) % PROT_FRAME_LEN_RECV] << 16) |
            (frame[(head_oft + LEN_INDEX_VAL + 3) % PROT_FRAME_LEN_RECV] << 24));    // 合成帧长度
}

/**
 * @brief   获取 crc-16 校验值
 * @param   *frame:  数据缓冲区.
 * @param   head_oft: 帧头的偏移位置
 * @param   head_oft: 帧长
 * @return  帧长度.
 */
static uint8_t get_frame_checksum(uint8_t *frame, uint16_t head_oft, uint16_t frame_len)
{
    return (frame[(head_oft + frame_len - 1) % PROT_FRAME_LEN_RECV]);
}

/**
 * @brief   查找帧头
 * @param   *buf:  数据缓冲区.
 * @param   ring_buf_len: 缓冲区大小
 * @param   start: 起始位置
 * @param   len: 需要查找的长度
 * @return  -1：没有找到帧头，其他值：帧头的位置.
 */
static int32_t recvbuf_find_header(uint8_t *buf, uint16_t ring_buf_len, uint16_t start, uint16_t len)
{
    uint16_t i = 0;

    for (i = 0; i < (len - 3); i++)
    {
        if (((buf[(start + i + 0) % ring_buf_len] <<  0) |
             (buf[(start + i + 1) % ring_buf_len] <<  8) |
             (buf[(start + i + 2) % ring_buf_len] << 16) |
             (buf[(start + i + 3) % ring_buf_len] << 24)) == FRAME_HEADER)
        {
            return ((start + i) % ring_buf_len);
        }
    }
    return -1;
}

/**
 * @brief   计算为解析的数据长度
 * @param   *buf:  数据缓冲区.
 * @param   ring_buf_len: 缓冲区大小
 * @param   start: 起始位置
 * @param   end: 结束位置
 * @return  为解析的数据长度
 */
static int32_t recvbuf_get_len_to_parse(uint16_t frame_len, uint16_t ring_buf_len,uint16_t start, uint16_t end)
{
    uint16_t unparsed_data_len = 0;

    if (start <= end)
        unparsed_data_len = end - start;
    else
        unparsed_data_len = ring_buf_len - start + end;

    if (frame_len > unparsed_data_len)
        return 0;
    else
        return unparsed_data_len;
}

/**
 * @brief   接收数据写入缓冲区
 * @param   *buf:  数据缓冲区.
 * @param   ring_buf_len: 缓冲区大小
 * @param   w_oft: 写偏移
 * @param   *data: 需要写入的数据
 * @param   *data_len: 需要写入数据的长度
 * @return  void.
 */
static void recvbuf_put_data(uint8_t *buf, uint16_t ring_buf_len, uint16_t w_oft,
        uint8_t *data, uint16_t data_len)
{
    if ((w_oft + data_len) > ring_buf_len)               // 超过缓冲区尾
    {
        uint16_t data_len_part = ring_buf_len - w_oft;     // 缓冲区剩余长度

        /* 数据分两段写入缓冲区*/
        memcpy(buf + w_oft, data, data_len_part);                         // 写入缓冲区尾
        memcpy(buf, data + data_len_part, data_len - data_len_part);      // 写入缓冲区头
    }
    else
        memcpy(buf + w_oft, data, data_len);    // 数据写入缓冲区
}

/**
 * @brief   查询帧类型（命令）
 * @param   *data:  帧数据
 * @param   data_len: 帧数据的大小
 * @return  帧类型（命令）.
 */
static uint8_t protocol_frame_parse(uint8_t *data, uint16_t *data_len)
{
    uint8_t frame_type = CMD_NONE;
    uint16_t need_to_parse_len = 0;
    int16_t header_oft = -1;
    uint8_t checksum = 0;
    
    need_to_parse_len = recvbuf_get_len_to_parse(parser.frame_len, PROT_FRAME_LEN_RECV, parser.r_oft, parser.w_oft);    // 得到为解析的数据长度
    if (need_to_parse_len < 9)     // 肯定还不能同时找到帧头和帧长度
        return frame_type;

    /* 还未找到帧头，需要进行查找*/
    if (0 == parser.found_frame_head)
    {
        /* 同步头为四字节，可能存在未解析的数据中最后一个字节刚好为同步头第一个字节的情况，
           因此查找同步头时，最后一个字节将不解析，也不会被丢弃*/
        header_oft = recvbuf_find_header(parser.recv_ptr, PROT_FRAME_LEN_RECV, parser.r_oft, need_to_parse_len);
        if (0 <= header_oft)
        {
            /* 已找到帧头*/
            parser.found_frame_head = 1;
            parser.r_oft = header_oft;
          
            /* 确认是否可以计算帧长*/
            if (recvbuf_get_len_to_parse(parser.frame_len, PROT_FRAME_LEN_RECV,
                    parser.r_oft, parser.w_oft) < 9)
                return frame_type;
        }
        else 
        {
            /* 未解析的数据中依然未找到帧头，丢掉此次解析过的所有数据*/
            parser.r_oft = ((parser.r_oft + need_to_parse_len - 3) % PROT_FRAME_LEN_RECV);
            return frame_type;
        }
    }
    
    /* 计算帧长，并确定是否可以进行数据解析*/
    if (0 == parser.frame_len) 
    {
        parser.frame_len = get_frame_len(parser.recv_ptr, parser.r_oft);
        if(need_to_parse_len < parser.frame_len)
            return frame_type;
    }

    /* 帧头位置确认，且未解析的数据超过帧长，可以计算校验和*/
    if ((parser.frame_len + parser.r_oft - PROT_FRAME_LEN_CHECKSUM) > PROT_FRAME_LEN_RECV)
    {
        /* 数据帧被分为两部分，一部分在缓冲区尾，一部分在缓冲区头 */
        checksum = check_sum(checksum, parser.recv_ptr + parser.r_oft, 
                PROT_FRAME_LEN_RECV - parser.r_oft);
        checksum = check_sum(checksum, parser.recv_ptr, parser.frame_len -
                PROT_FRAME_LEN_CHECKSUM + parser.r_oft - PROT_FRAME_LEN_RECV);
    }
    else 
    {
        /* 数据帧可以一次性取完*/
        checksum = check_sum(checksum, parser.recv_ptr + parser.r_oft, parser.frame_len - PROT_FRAME_LEN_CHECKSUM);
    }

    if (checksum == get_frame_checksum(parser.recv_ptr, parser.r_oft, parser.frame_len))
    {
        /* 校验成功，拷贝整帧数据 */
        if ((parser.r_oft + parser.frame_len) > PROT_FRAME_LEN_RECV) 
        {
            /* 数据帧被分为两部分，一部分在缓冲区尾，一部分在缓冲区头*/
            uint16_t data_len_part = PROT_FRAME_LEN_RECV - parser.r_oft;
            memcpy(data, parser.recv_ptr + parser.r_oft, data_len_part);
            memcpy(data + data_len_part, parser.recv_ptr, parser.frame_len - data_len_part);
        }
        else 
        {
            /* 数据帧可以一次性取完*/
            memcpy(data, parser.recv_ptr + parser.r_oft, parser.frame_len);
        }
        *data_len = parser.frame_len;
        frame_type = get_frame_type(parser.recv_ptr, parser.r_oft);

        /* 丢弃缓冲区中的命令帧*/
        parser.r_oft = (parser.r_oft + parser.frame_len) % PROT_FRAME_LEN_RECV;
    }
    else
    {
        /* 校验错误，说明之前找到的帧头只是偶然出现的废数据*/
        parser.r_oft = (parser.r_oft + 1) % PROT_FRAME_LEN_RECV;
    }
    parser.frame_len = 0;
    parser.found_frame_head = 0;

    return frame_type;
}

/**
 * @brief   接收数据处理
 * @param   *data:  要计算的数据的数组.
 * @param   data_len: 数据的大小
 * @return  void.
 */
void protocol_data_recv(uint8_t *data, uint16_t data_len)
{
    recvbuf_put_data(parser.recv_ptr, PROT_FRAME_LEN_RECV, parser.w_oft, data, data_len);    // 接收数据
    parser.w_oft = (parser.w_oft + data_len) % PROT_FRAME_LEN_RECV;                          // 计算写偏移
}

/**
 * @brief   初始化接收协议
 * @param   void
 * @return  初始化结果.
 */
int32_t protocol_init(void)
{
    memset(&parser, 0, sizeof(struct prot_frame_parser_t));
    
    /* 初始化分配数据接收与解析缓冲区*/
    parser.recv_ptr = recv_buf;
  
    return 0;
}

/**
 * @brief   接收的数据处理
 * @param   void
 * @return  -1：没有找到一个正确的命令.
 */
int8_t receiving_process(void)
{
  uint8_t frame_data[128];         // 要能放下最长的帧
  uint16_t frame_len = 0;          // 帧长度
  uint8_t cmd_type = CMD_NONE;     // 命令类型
  packet_head_t packet;
  
  while(1)
  {
		cmd_type = protocol_frame_parse(frame_data, &frame_len);
		packet.ch = frame_data[CHX_INDEX_VAL];
    switch (cmd_type)
    {
      case CMD_NONE:
      {
        return -1;
      }
// 设置 P I D
      case SET_P_I_D_CMD:
      {
        type_cast_t temp_p, temp_i, temp_d;          
        temp_p.i = COMPOUND_32BIT(&frame_data[13]);
        temp_i.i = COMPOUND_32BIT(&frame_data[17]);
        temp_d.i = COMPOUND_32BIT(&frame_data[21]);
        
        set_p_i_d(packet.ch, temp_p.f, temp_i.f, temp_d.f);    
				OLED_ShowNum(3, 1, (uint32_t)(temp_p.f*1000),4);
				OLED_ShowNum(3, 6, (uint32_t)(temp_i.f*1000),4);
				OLED_ShowNum(3, 11, (uint32_t)(temp_d.f*1000),4);
				HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_2);
      }break;
// 设置目标值
      case SET_TARGET_CMD:
      {
				type_cast_t temp_target;
        temp_target.f = COMPOUND_32BIT(&frame_data[13]);    // 得到数据
				
				set_rpm(packet.ch - 1, temp_target.f);
				pid_reset(packet.ch,(uint16_t)temp_target.f);
				
				Ctrl_Para.enable_fan[packet.ch] = 1;
				OLED_ShowNum(packet.ch, 1, (uint16_t)temp_target.f, 5);
				HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_2);
      }break;
// 启动电机
      case START_CMD:
      {
        Ctrl_Para.enable_fan[packet.ch] = 1;				
       // set_rpm(packet.ch,(float)START_RPM2);
				//pid_reset(packet.ch,0);
				//pid_enable[0] = 1;
				HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_2);  
      }break;
// 停止电机      
      case STOP_CMD:
      {
        Ctrl_Para.enable_fan[packet.ch] = 0;
				/*pid_reset(packet.ch,0);
				pid_enable[0] = 0;*/
        set_rpm(packet.ch, 0);
				HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_2);        
      }break;
// 使能PID
			case PIDSTART_CMD:
      {
       	Ctrl_Para.enable_pid[0] = 1;
				OLED_ShowString(4,1,"Close");
				HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_2);  
      }break;
// 失能PID
			case PIDSTOP_CMD:
      {
       	Ctrl_Para.enable_pid[0] = 0;
				OLED_ShowString(4,1,"Open");
				HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_2);         
      }break;
// 复位系统      
      case RESET_CMD:
      {
        HAL_NVIC_SystemReset();          
      }break;
// 设置sin参数
			case SET_SIN_CMD:
			{
				type_cast_t temp_mid, temp_period, temp_amp, temp_ctrl_num;
				temp_mid.f = COMPOUND_32BIT(&frame_data[13]);
        temp_period.i = COMPOUND_32BIT(&frame_data[17]);
        temp_amp.f = COMPOUND_32BIT(&frame_data[21]);
				temp_ctrl_num.i = COMPOUND_32BIT(&frame_data[25]);
				Sin_Para1.mid = temp_mid.f;
				Sin_Para1.period = temp_period.i;
				Sin_Para1.amplitude = temp_amp.f;
				Sin_Para1.ctrl_num = temp_ctrl_num.i;

				HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_2);
			}break;
// 使能&失能SIN控制
			case SIN_ENABLE_CMD:
			{
				type_cast_t temp_sin_enable;
        temp_sin_enable.i = COMPOUND_32BIT(&frame_data[13]);
				if(temp_sin_enable.i == 1){
					TrackCurve_Para.start_cacul = 1;
				}else{
					TrackCurve_Para.enable_TrackCurve = 0;
					TrackCurve_Para.start_cacul = 0;
				}
				OLED_ShowString(4, 16, "F");
				HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_2);
			}break;
// 周期数
      case SET_PERIOD_CMD:
      {
				uint32_t temp_ts = COMPOUND_32BIT(&frame_data[13]);	//闭环控制采样时间ms
				uint32_t temp_pwmarr = COMPOUND_32BIT(&frame_data[17]); //pwm输出ARR值
				SamplingTime_Modify((uint16_t)temp_ts, (uint16_t)temp_pwmarr);
				
				HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_2);
      }break;
//使能转速同步
			case SEND_ENABLE:
			{
				type_cast_t temp_send_enable;
        temp_send_enable.i = COMPOUND_32BIT(&frame_data[13]);
				if(temp_send_enable.i == 1){
					Sys_Para.enable_send = 1;
					//TrackCurve_Para.enable_TrackCurve = 1;
				}else{
					Sys_Para.enable_send = 0;
					//TrackCurve_Para.enable_TrackCurve = 0;
				}
				HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_2);
			}break;
      default: 
        return -1;
    }
  }
}

/**
* @brief 上传数据
  * @param cmd：命令
  * @param ch: 曲线通道
  * @param data：参数指针
  * @param num：参数个数
  * @retval 无
  */
void send_computer(uint8_t cmd, uint8_t ch, void *data, uint8_t num)
{
  //uint8_t sum = 0;    // 校验和
  //num *= 4;           // 一个参数 4 个字节
  
  static head_s set_packet;
  
  set_packet.head = FRAME_HEADER;     // 包头 0x59485A53
 // set_packet.len  = 0x0F;      // 包长
  set_packet.ch   = ch;              // 设置通道
  set_packet.cmd  = cmd;             // 设置命令
  
  //sum = check_sum(0, (uint8_t *)&set_packet, sizeof(set_packet));       // 计算包头校验和
  //sum = check_sum(sum, (uint8_t *)data, num);                           // 计算参数校验和
  
  HAL_UART_Transmit(&huart3, (uint8_t *)&set_packet, sizeof(set_packet), 0xFFFFF);    // 发送数据头
  HAL_UART_Transmit(&huart3, (uint8_t *)data, num, 0xFFFFF);                          // 发送参数
  //HAL_UART_Transmit(&huart3, (uint8_t *)&sum, sizeof(sum), 0xFFFFF);                  // 发送校验和
}

/**********************************************************************************************/
