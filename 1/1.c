#include "BF7006AMxx_config.h"
#include "SEGGER_RTT.h"

#define DEFINE_VARIABLES

#include "xc.h"

//void lin_handler(lin_slave_task_struct *slave_task);
//void lin_revhandler(lin_slave_task_struct *slave_task);

/**
 * @brief 毫秒级延时函数
 * @param t 延时时间(毫秒)
 * @note 使用系统定时器实现精确延时，当t=0时执行空循环延时
 */
void mdelay(uint16_t t)
{
    SysOpt.wait = t;

    if (t == 0)
    {
        for (int j = 0; j < 1000; j++)
        {
            ;
            ;
        }
        return;
    }

    while (SysOpt.wait)
    {
    }
}

/**
 * @brief 主函数 - 系统初始化和主循环
 * @return int 程序返回值(通常不返回)
 * @note 系统启动入口，完成硬件初始化、外设配置和主任务循环
 */
int main(void)
{

	
    config_return_status init_ret_val;
    Disable_Irq();                    // 禁用中断
    SysTick_Config(16000);            // 配置系统定时器，1ms中断
    init_ret_val = peripherals_config(); // 初始化外设
		SysOpt.kelincanid[1] = 0X19FF1193; // 设置默认CAN ID
    if (is_xtal_init_success == XTAL_Init_Status_Init_Failed)
    {
        // 晶振初始化失败处理
    }

    if (init_ret_val.overall == ERROR)
    {
        // 外设初始化失败处理
    }
#ifndef USE_WDT
    wdt_config(0x0000);              // 配置看门狗
    wdt_updata(0x0000);              // 更新看门狗
#endif
    Enable_Irq();                     // 启用中断

#ifdef USE_LIN
    ////lin_init();                   // LIN总线初始化(如果启用)
#endif

#ifdef DEBUG_SEGG
    //SEGGER_RTT_Init();              // SEGGER RTT调试初始化
    //SEGGER_RTT_SetTerminal(0);      // 设置RTT终端
#endif

    ClrMem();                         // 清除系统内存
    ClrMemUds();                      // 清除UDS相关内存

#ifdef USE_LIN
    LinResponseCallBack = lin_handler;    // 设置LIN响应回调
    LinReceiveCallBack = lin_revhandler;  // 设置LIN接收回调
#endif

    PtcClose();                       // 关闭PTC加热器

    while (1)                         // 主循环
    {

#ifdef USE_WDT
        wdt_clear();                  // 清除看门狗计数器(2秒超时)
#endif
			
        AdcCheck();                   // 检查ADC转换
        PtcCheck();                   // 检查PTC状态

#ifdef USE_LIN
        CheckLinTx();                 // 检查LIN发送(如果使用LIN)
#else
        CheckCanTx();                 // 检查CAN发送
        CheckCanRx();                 // 检查CAN接收
#endif

#ifdef USE_BUSOFF
//        CheckCanBusOff();           // 检查CAN总线关闭状态
#endif

//        uds_cf();                   // UDS控制流
//        uds_dtc();                  // UDS诊断故障码
//        uds_session();              // UDS会话管理
    }
}

/**
 * @brief 清除系统内存
 * @note 将系统选项结构体、工作状态结构体和错误标志清零
 */
void ClrMem(void)
{
    uint8_t *p;
    int i;

    // 清除系统选项结构体
    p = (uint8_t *)&SysOpt;
    for (i = 0; i < sizeof(SYS_OPT_T); i++)
    {
        *p++ = 0;
    }

    // 清除工作状态结构体
    p = (uint8_t *)&WorkStat;
    for (i = 0; i < sizeof(WORK_STAT_T); i++)
    {
        *p++ = 0;
    }

    // 清除错误标志
    ErrorFlag.errcode[0] = 0;
    ErrorFlag.errcode[1] = 0;
    ErrorFlag.errcode[2] = 0;
    ErrorFlag.errcode[3] = 0;
}

//------------------------------------------------------
// HVH_Rsp_1：0X19FF1193/0X19FF1293/0X19FF1393/0X19FF1493
//------------------------------------------------------
/**
 * @brief 共享标准输出函数1 - 构建PTC状态数据包
 * @param tx 输出数据缓冲区指针
 * @note 根据不同的ID1/ID2组合，构建不同CAN ID的PTC状态数据包
 *       包含PTC功率、状态、错误码、温度、电流、电压等信息
 */
void Sharing_Stand_Out1(uint8_t *tx)
{


		ERR_FLAG_T tmperr;
		unsigned char level;
		unsigned char errcode;	
	
    uint8_t PTCDutyCycle = 0; // PTC占空比	
	
    // 复制错误标志到临时变量
    tmperr.errcode[0] = ErrorFlag.errcode[0];
    tmperr.errcode[1] = ErrorFlag.errcode[1];
    tmperr.errcode[2] = ErrorFlag.errcode[2];
    tmperr.errcode[3] = ErrorFlag.errcode[3];
 

	  tx[0]=0;
  	if (WorkStat.runflag == 1)
		tx[0] = SysOpt.vcupower;  // VCU功率设置

//BIT7~0 固定值 0X8C
//分辨率 0.1KW
//例如：14KW PTC,发送0X8C	
	tx[1] = 0x64;  // PTC功率固定值

	if (WorkStat.runflag == 1)
	{
		tx[2] = WorkStat.ptcpower / 100;  // 实际PTC功率(0.1KW分辨率)
		if(WorkStat.ptcpower > 8500)
		{
			tx[2] = (WorkStat.ptcpower + 100)/ 100;  // 功率补偿
		}
	}
	else
	{
		tx[2] = 0;  // 未运行时功率为0
	}

	errcode = 0;
	
	tx[3] = 0;
	if (WorkStat.runflag == 1)
	{
		tx[3] = 1;  // 运行状态
	}
	else if (tmperr.errcode[0] != 0 || tmperr.errcode[1] != 0 || tmperr.errcode[2] != 0 || tmperr.errcode[3] != 0)
	{
		tx[3] = 3;  // 错误状态
	}
	
	// 检查欠压故障
	if (tmperr.undervoltage==1 && SysOpt.vcuen==1)
	{
		errcode = 0x04;
		level = 1;
	}
	
	// 检查过压故障
	if (tmperr.overvoltage)
	{
		errcode = 0x02;
		level = 2;
	}

	// 检查NTC温度报警(85度)
	if (tmperr.ntc1alarm0 || tmperr.ntc3alarm0)
	{
		errcode = 0x10;
		level = 2;
	}

	// 检查NTC温度报警(95度)
	if (tmperr.ntc1alarm1 || tmperr.ntc3alarm1)
	{
		errcode = 0x10;
		level = 2;
	}

	// 检查NTC温度报警(105度)
	if (tmperr.ntc1alarm2 || tmperr.ntc3alarm2)
	{
		errcode = 0x10;
		level = 3;
	}

	// 检查CAN通信错误
	if (tmperr.canerr)
	{
		errcode = 0x20; //reserved bit
		level = 1;
	}
	
	// 检查其他故障
	if (tmperr.ntc1err || tmperr.ntc3err  || tmperr.drvfault || tmperr.hv15outofrange || tmperr.lvoutofrange)
	{
		errcode = 0x01;
		level = 2;
	}
	
	// 检查IGBT短路故障
	if (tmperr.igbtshort)
	{
		errcode = 0x08;
		level = 3;
	}
	
	tx[4] = errcode;  // 错误码
	tx[5] = WorkStat.ntc3;  // NTC3温度

	// PTC高压电流
	if (WorkStat.runflag == 1)
	{
		if (WorkStat.ptchvcurrent > 510)
		{
			tx[6] = 255;  // 电流限制
		}
		else
		{
			tx[6] = WorkStat.ptchvcurrent / 2;  // 电流值(分辨率0.5A)

			if(WorkStat.ptcpower > 8500)
			{
					tx[6] =(WorkStat.ptchvcurrent + (610 / WorkStat.ptchvvoltage))/ 2;  // 功率补偿
			}			
		}
	}
	else
	{
		tx[6] = 0;  // 未运行时电流为0
	}

	tx[7] = WorkStat.ptchvvoltage / 4;  // PTC高压电压(分辨率0.25V)
	
//		tx[0] = SysOpt.advalue_lv   / 20;
//		tx[1] = SysOpt.advalue_ntc1 / 20;
//		tx[2] = SysOpt.advalue_ntc2 / 20;
//		tx[3] = tmperr.errcode[0];
//		tx[4] = tmperr.errcode[1];
//		tx[5] = tmperr.errcode[2];
//		tx[6] = tmperr.errcode[3];
//		tx[7] = tmperr.errcode[2];
		
	
}




#ifndef USE_LIN

/**
 * @brief 检查CAN发送函数
 * @note 根据定时器超时发送PTC状态数据，根据ID1/ID2组合选择不同的CAN ID
 */
void CheckCanTx(void)
{
 	static uint8_t tick = 0;
	uint8_t tx[8];

	if (SysOpt.cantxtimeout == CAN_TIMEOUT)  // CAN发送超时检查
	{
		SysOpt.cantxtimeout = 0;

		Sharing_Stand_Out1(tx);  // 构建发送数据
		
		// 根据ID1/ID2组合选择不同的CAN ID
		if(ID1 == 1 && ID2 == 1)
		{
			SysOpt.kelincanid[1] = 0X19FF1193;
		}
		else if (ID1 == 0 && ID2 == 0)
		{
			SysOpt.kelincanid[1] = 0X19FF1293;
		}
		else if (ID1 == 1 && ID2 == 0)
		{
			SysOpt.kelincanid[1] = 0X19FF1393;		
		}
		else if(ID1 == 0 && ID2 == 1)
		{
			SysOpt.kelincanid[1] = 0X19FF1493;
		}		
		
		CAN_Send_Message(SysOpt.kelincanid[1], CanDataFormat, 8, tx);  // 发送CAN消息
	}
}

/**
 * @brief 检查CAN接收函数
 * @note 处理接收到的CAN消息，包括控制命令和调试命令
 */
void CheckCanRx(void)
{
	uint8_t tx[8];
	uint8_t i;

	if (SysOpt.canrxflag == 1)  // 接收到CAN消息
	{
		SysOpt.canrxflag = 0;
		ErrorFlag.canerr = 0;
		SysOpt.canrxtimeout = 0;

		// 根据ID1/ID2组合解析不同位置的VCU功率和使能信号
		if(ID1 == 1 && ID2 == 1)
		{
		SysOpt.vcupower = SysOpt.canrxbuf[0];
		SysOpt.vcuen = (SysOpt.canrxbuf[1] & 0x03);
		}
		else if (ID1 == 0 && ID2 == 0)
		{
		SysOpt.vcupower = SysOpt.canrxbuf[2];
		SysOpt.vcuen = (SysOpt.canrxbuf[3] & 0x03);
		}
		else if (ID1 == 1 && ID2 == 0)
		{
		SysOpt.vcupower = SysOpt.canrxbuf[4];
		SysOpt.vcuen = (SysOpt.canrxbuf[5] & 0x03);	
		}
		else if(ID1 == 0 && ID2 == 1)
		{
		SysOpt.vcupower = SysOpt.canrxbuf[6];
		SysOpt.vcuen = (SysOpt.canrxbuf[7] & 0x03);
		}
		

  }

    // 调试命令处理，0x222 ID
    if(SysOpt.canrxflag1 == 1)
    {
        SysOpt.canrxflag1 = 0;

        // 虚拟高压电压设置命令
        if (SysOpt.canrxbuf1[0] == 0xAA && SysOpt.canrxbuf1[1] == 0x55 && SysOpt.canrxbuf1[2] == 0x33 && SysOpt.canrxbuf1[3] == 0x44)
        {
            WorkStat.ptchvsim_debug = 1;
        }
        // 调试数据请求命令
        else if (SysOpt.canrxbuf1[0] == 0xAA && SysOpt.canrxbuf1[1] == 0x55 && SysOpt.canrxbuf1[2] == 0x03 && SysOpt.canrxbuf1[3] == 0xfc)
        {
            tx[0] = WorkStat.ntc1;           // NTC1温度
            tx[1] = WorkStat.ntc3;           // NTC3温度
            tx[2] = ErrorFlag.errcode[0];    // 错误码0
            tx[3] = ErrorFlag.errcode[1];    // 错误码1
            tx[4] = ErrorFlag.errcode[2];    // 错误码2
            tx[5] = WorkStat.igbttemp;       // IGBT温度
            tx[6] = WorkStat.lv;             // 低压电压
            tx[7] = WorkStat.hv15;           // 15V电压
            CAN_Send_Message(0x10000272, 1, 8, tx);  // 发送调试数据
        }
		// LED控制命令
		else if (SysOpt.canrxbuf1[0] == 0xAA && SysOpt.canrxbuf1[1] == 0x55 && SysOpt.canrxbuf1[2] == 0x04 && SysOpt.canrxbuf1[3] == 0xfb)
		{
#if defined(W01)
			if (SysOpt.canrxbuf1[4] == 1)
			{
					SET_LED_ALARM;  // 设置报警LED
			}
			if (SysOpt.canrxbuf1[4] == 0)
			{
				CLR_LED_ALARM;  // 清除报警LED
			}
#endif
		}
        // 系统重启命令
        else if (SysOpt.canrxbuf1[0] == 0xAA && SysOpt.canrxbuf1[1] == 0x55 && SysOpt.canrxbuf1[2] == 0x02 && SysOpt.canrxbuf1[3] == 0xfd)
        {
            PtcClose();  // 关闭PTC
            while (1)
            {
                ;
                ; // 等待看门狗重启
            }
        }
    }
}

/**
 * @brief 检查CAN总线关闭状态
 * @note 处理CAN总线关闭故障，实现自动恢复机制
 */
void CheckCanBusOff(void)
{
    static uint16_t t = 50;
    static uint8_t busofftime=0;

    if (SysOpt.can_busoffhold == 1) // 检测到总线关闭
    {
        SysOpt.can_busoffhold = 0;

        if (busofftime < 100)
        {
            busofftime++; // 总线关闭计数器递增
        }

        if (busofftime <= 5)
        {
            t = 100; // 100ms快速恢复
        }
        else
        {
            t = 1000; // 1000ms慢速恢复
        }

				SysOpt.vcuen = 0;  // 关闭VCU使能
				
        SysOpt.cantxtimeout = 0;
        SysOpt.canrxtimeout = 0;
        SysOpt.canbufofftimeout = 0;
        SysOpt.canbusoffflag = 1;

        SysOpt.can_txerrstat = 0; // 清除CAN发送状态
        SysOpt.can_buscoffcnt++;
        can_config(); // 重新配置CAN设置

        // 总线关闭次数>3且ACC开启2秒且12V正常时记录DTC
        if (SysOpt.can_buscoffcnt >= 3)
        {
            if (UdsOpt.accon_2s == 1 && ErrorFlag.lvoutofrange == 0)
            {
                if (UdsOpt.busoffflag == 0) // 从无到有
                    UdsOpt.busoffflag = 1;    // 记录DTC
            }
        }
    }

    if (SysOpt.canbufofftimeout < 20000) //
        SysOpt.canbufofftimeout++;

    if (SysOpt.canbufofftimeout < t && SysOpt.canbusoffflag == 1)
    {
        SysOpt.cantxtimeout = 0; // 取消发送清除
        SysOpt.canrxtimeout = 0; // 取消接收清除
    }

    if (SysOpt.canbufofftimeout >= t && SysOpt.canbusoffflag == 1)
    {
        SysOpt.canbufofftimeout = 0;
        SysOpt.canbusoffflag = 0;
        SysOpt.canrxtimeout = 0;
        SysOpt.cantxtimeout = CAN_TIMEOUT - 1; // 立即发送
    }
}

#else

/**
 * @brief 检查LIN发送函数
 * @note 根据定时器超时发送PTC状态数据到LIN总线
 */
void CheckLinTx(void)
{
    if (SysOpt.cantxtimeout == CAN_TIMEOUT)
    {
        SysOpt.cantxtimeout = 0;

        Sharing_Stand_Out1(SysOpt.lintx_buf);  // 构建LIN发送数据
    }
}

/**
 * @brief LIN接收处理函数
 * @param slave_task LIN从机任务结构体指针
 * @note 处理接收到的LIN消息，包括控制命令和CRC校验
 */
void lin_revhandler(lin_slave_task_struct *slave_task)
{
    uint8_t crc;
    static uint8_t crc_error_cnt;

    if (slave_task == NULL)
    {
        return;
    }
    else if (slave_task->state == LIN_STATE_CHECKSUM)
    {

        if (slave_task->rece_pid == 0x3c)  // 调试命令PID
        {
            // 调试 设置虚拟电压 3C ：aa 55 33 44 ff ff ff ff
            if (slave_task->rece_dat[0] == 0xaa && slave_task->rece_dat[1] == 0x55 && slave_task->rece_dat[2] == 0x33 && slave_task->rece_dat[3] == 0x44 &&
                    slave_task->rece_dat[4] == 0xff && slave_task->rece_dat[5] == 0xff && slave_task->rece_dat[6] == 0xff && slave_task->rece_dat[7] == 0xff)
            {
                WorkStat.ptchvsim_debug = 1;  // 启用虚拟高压电压
            }

            // 关闭PTC命令
            if (slave_task->rece_dat[0] == 0x00 && slave_task->rece_dat[1] == 0xff && slave_task->rece_dat[2] == 0xff && slave_task->rece_dat[3] == 0xff &&
                    slave_task->rece_dat[4] == 0xff && slave_task->rece_dat[5] == 0xff && slave_task->rece_dat[6] == 0xff && slave_task->rece_dat[7] == 0xff)
            {
                if (WorkStat.runflag == 1)
                {
                    PtcClose();  // 关闭PTC
                    SysOpt.vcuen = 0;
                    SysOpt.vcupower = 0;
                }
            }
        }
        else
        {
            if(slave_task->rece_pid == 0xa6)  // 控制命令PID
            {
                crc = lin_checksum(0xa6, slave_task->rece_dat, 8); // 0xa6 对应ID ：26

                if (crc == slave_task->rece_checksum)  // CRC校验正确
                {
                    crc_error_cnt = 0;
                    ErrorFlag.canerr = 0;
                    SysOpt.canrxflag = 0;
                    ErrorFlag.canerr = 0;
                    SysOpt.canrxtimeout = 0;

                    // 接收占空比LIN信号
                    if (slave_task->rece_dat[4] < 10)
                    {
                        SysOpt.vcupower = 0;  // 占空比小于10%时关闭
                    }
                    else if (slave_task->rece_dat[4] > 95)
                    {
                        SysOpt.vcupower = 100;  // 占空比大于95%时全功率
                    }
                    else
                    {
                        SysOpt.vcupower = slave_task->rece_dat[4];  // 设置占空比
                    }

                    // 接收开关使能LIN信号
                    SysOpt.vcuen = (slave_task->rece_dat[6] & 0x01);
                }
                else // CRC错误处理
                {
                    if(crc_error_cnt < 10)
                    {
                        crc_error_cnt ++;
                    }
                    else
                    {
                        ErrorFlag.canerr = 1;  // CRC错误10次后报故障
                    }
                }
            }
        }
    }
}

/**
 * @brief LIN响应处理函数
 * @param slave_task LIN从机任务结构体指针
 * @note 根据接收到的PID准备相应的响应数据
 */
void lin_handler(lin_slave_task_struct *slave_task)
{
    if (slave_task == NULL)
    {
        return;
    }

    if (slave_task->rece_pid == lin_pid_calc(0x26))  // 控制命令PID
    {
        slave_task->is_tx = Node_Rx;  // 设置为接收模式
        slave_task->data_lenth = 8;
    }

    if (slave_task->rece_pid == 0x3c)  // 调试命令PID
    {
        slave_task->is_tx = Node_Rx;  // 设置为接收模式
        slave_task->data_lenth = 8;
    }

    /* pid匹配 */
    // 需要发送8字节数据
    else if (slave_task->rece_pid == lin_pid_calc(0x27))  // 状态数据PID
    {
        slave_task->is_tx = Node_Tx;  // 设置为发送模式
        slave_task->data_lenth = 8;

        // 复制PTC状态数据到响应缓冲区
        slave_task->response_dat[0] = SysOpt.lintx_buf[0];
        slave_task->response_dat[1] = SysOpt.lintx_buf[1];
        slave_task->response_dat[2] = SysOpt.lintx_buf[2];
        slave_task->response_dat[3] = SysOpt.lintx_buf[3];
        slave_task->response_dat[4] = SysOpt.lintx_buf[4];
        slave_task->response_dat[5] = SysOpt.lintx_buf[5];
        slave_task->response_dat[6] = SysOpt.lintx_buf[6];
        slave_task->response_dat[7] = SysOpt.lintx_buf[7];

        slave_task->response_checksum = lin_checksum(0xe7, slave_task->response_dat, 8);  // 计算响应CRC
    }
    else if (slave_task->rece_pid == lin_pid_calc(0x28))  // 调试数据PID
    {
        slave_task->is_tx = Node_Tx;  // 设置为发送模式
        slave_task->data_lenth = 8;

        // 准备调试数据
        slave_task->response_dat[0] = WorkStat.ntc1;           // NTC1温度
        slave_task->response_dat[1] = WorkStat.ntc3;           // NTC3温度
        slave_task->response_dat[2] = ErrorFlag.errcode[0];    // 错误码0
        slave_task->response_dat[3] = ErrorFlag.errcode[1];    // 错误码1
        slave_task->response_dat[4] = WorkStat.igbttemp;       // IGBT温度
        slave_task->response_dat[5] = WorkStat.hv15;           // 15V电压
        slave_task->response_dat[6] = WorkStat.lv;             // 低压电压
        slave_task->response_dat[7] = WorkStat.ptccurrent_debug; // PTC电流调试值

        slave_task->response_checksum = lin_checksum(0xa8, slave_task->response_dat, 8);  // 计算响应CRC
    }
}


#endif

//--------------------------END OF FILE-----------------------------------
