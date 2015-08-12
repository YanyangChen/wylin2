#ifndef __RETURN_CODE_H_
#define __RETURN_CODE_H_

#define		RTN_WAITTING_CMD					-1	///< µÈŽýÖžÁîÍê³É

#define		RTN_OK								0	///< ·µ»Ø³É¹Š
#define		RTN_ERROR							1	///< ·µ»ØŽíÎó£¬ÓÃÓÚ²»ÐèÒªÖž³öŽíÎóÀàÐÍµÄ·µ»Ø
#define		RTN_CREATE_SHARE_MEMORY_FAIL		2	///< ŽŽœš¹²ÏíÄÚŽæÊ§°Ü£¬CShareMemory::OpenShareMemoryº¯Êý·µ»Ø
#define		RTN_COMM_ERROR_CONNECT				3	///< ÍšÑ¶Á¬œÓ³öŽí
#define		RTN_COMM_SPLIT_READ_TIMEOUT			4	///< ÍšÑ¶¶ÁÈ¡³¬Ê±
#define		RTN_INVALID_COMMAND					5	///< ÎÞÐ§ÖžÁî
#define		RTN_UNKNOWN_COMMAND					6	///< ÎŽÖªÖžÁî
#define		RTN_INVALID_COMMAND_PARAMS			7	///< ÎÞÐ§µÄÖžÁî²ÎÊý
#define		RTN_COMM_ERROR_TIMEOUT				8	///< ÍšÐí³¬Ê±
#define		RTN_SHARE_MEMORY_NOT_EXIST			9	///< ¹²ÏíÄÚŽæ²»ŽæÔÚ
#define		RTN_TASK_IS_BUSY					10	///< ÈÎÎñÎŽÖŽÐÐÍê±Ï
#define		RTN_KERNEL_IS_BUSY					11	///< ÄÚºËÎŽÖŽÐÐÍê±Ï
#define		RTN_AXIS_NUMBER_ERROR				12	///< ÖáºÅ³¬³ö·¶Î§
#define		RTN_TASK_NOT_EXIST					13	///< ÈÎÎñ²»ŽæÔÚ
#define		RTN_POINTER_NULL					14	///< ÖžÕë²»ŽæÔÚ
#define		RTN_DATA_DIM_ERROR					15	///< ÊýŸÝÎ¬Êý²»¶Ô
#define		RTN_LINEAR_PLAN_FULL				16	///< Ö±ÏßÔË¶¯¹æ»®»º³åÂú
#define		RTN_RUNNING_IN_SERVO_OFF			17
#define		RTN_JOINTS_LIMITS_ERROR				18

#endif //__RETURN_CODE_H_
