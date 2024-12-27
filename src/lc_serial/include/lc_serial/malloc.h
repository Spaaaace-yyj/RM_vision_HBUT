#ifndef __MALLOC_H
#define __MALLOC_H
// #include "main.h"
#ifndef NULL
#define NULL 0
#endif

#define uint8_tc unsigned char
#define uint16_tc unsigned short


//仅使用SRAM1 = 176K，其他部分太小了，在不缺mem的情况下不使用；
#define MEM_BLOCK_SIZE			64  	  						//内存块大小为64字节
#define MEM_MAX_SIZE			  32*1024  						//最大管理内存 150K
#define MEM_ALLOC_TABLE_SIZE	MEM_MAX_SIZE/MEM_BLOCK_SIZE 	//内存表大小 <3K * 4

//内存管理控制器
struct _m_mallco_dev
{
	void (*init)();				//初始化
	uint16_tc (*perused)();		//内存使用率
	uint8_tc 	*membase;				//内存池 管理SRAMBANK个区域的内存
	unsigned int*memmap;					//内存管理状态表
	uint8_tc  memrdy;					//内存管理是否就绪
};
extern struct _m_mallco_dev mallco_dev;	 		//在mallco.c里面定义

void mymemset(void *s, uint8_tc c, unsigned int count);		//设置内存
void mymemcpy(void *des, void *src, unsigned int n);		//复制内存     
void my_mem_init(void);										//内存管理初始化函数(外/内部调用)
unsigned int my_mem_malloc(unsigned int size);					//内存分配(内部调用)
uint8_tc my_mem_free(unsigned int offset);					//内存释放(内部调用)
uint16_tc my_mem_perused(void) ;								//获得内存使用率(外/内部调用)

////////////////////////////////////////////////////////////////////////////////
//用户调用函数
void myfree(void *ptr);  						//内存释放(外部调用)
void *mymalloc(unsigned int size);					//内存分配(外部调用)
void *myrealloc(void *ptr,unsigned int size);		//重新分配内存(外部调用)
#endif

