#define nx_struct struct
#define nx_union union
#define dbg(mode, format, ...) ((void)0)
#define dbg_clear(mode, format, ...) ((void)0)
#define dbg_active(mode) 0
# 38 "/opt/msp430/msp430/include/sys/inttypes.h"
typedef signed char int8_t;
typedef unsigned char uint8_t;

typedef int int16_t;
typedef unsigned int uint16_t;

typedef long int32_t;
typedef unsigned long uint32_t;

typedef long long int64_t;
typedef unsigned long long uint64_t;




typedef int16_t intptr_t;
typedef uint16_t uintptr_t;
# 290 "/usr/lib/ncc/nesc_nx.h"
typedef struct { char data[1]; } __attribute__((packed)) nx_int8_t;typedef int8_t __nesc_nxbase_nx_int8_t  ;
typedef struct { char data[2]; } __attribute__((packed)) nx_int16_t;typedef int16_t __nesc_nxbase_nx_int16_t  ;
typedef struct { char data[4]; } __attribute__((packed)) nx_int32_t;typedef int32_t __nesc_nxbase_nx_int32_t  ;
typedef struct { char data[8]; } __attribute__((packed)) nx_int64_t;typedef int64_t __nesc_nxbase_nx_int64_t  ;
typedef struct { char data[1]; } __attribute__((packed)) nx_uint8_t;typedef uint8_t __nesc_nxbase_nx_uint8_t  ;
typedef struct { char data[2]; } __attribute__((packed)) nx_uint16_t;typedef uint16_t __nesc_nxbase_nx_uint16_t  ;
typedef struct { char data[4]; } __attribute__((packed)) nx_uint32_t;typedef uint32_t __nesc_nxbase_nx_uint32_t  ;
typedef struct { char data[8]; } __attribute__((packed)) nx_uint64_t;typedef uint64_t __nesc_nxbase_nx_uint64_t  ;



typedef struct { char data[1]; } __attribute__((packed)) nxle_int8_t;typedef int8_t __nesc_nxbase_nxle_int8_t  ;
typedef struct { char data[2]; } __attribute__((packed)) nxle_int16_t;typedef int16_t __nesc_nxbase_nxle_int16_t  ;
typedef struct { char data[4]; } __attribute__((packed)) nxle_int32_t;typedef int32_t __nesc_nxbase_nxle_int32_t  ;
typedef struct { char data[8]; } __attribute__((packed)) nxle_int64_t;typedef int64_t __nesc_nxbase_nxle_int64_t  ;
typedef struct { char data[1]; } __attribute__((packed)) nxle_uint8_t;typedef uint8_t __nesc_nxbase_nxle_uint8_t  ;
typedef struct { char data[2]; } __attribute__((packed)) nxle_uint16_t;typedef uint16_t __nesc_nxbase_nxle_uint16_t  ;
typedef struct { char data[4]; } __attribute__((packed)) nxle_uint32_t;typedef uint32_t __nesc_nxbase_nxle_uint32_t  ;
typedef struct { char data[8]; } __attribute__((packed)) nxle_uint64_t;typedef uint64_t __nesc_nxbase_nxle_uint64_t  ;
# 151 "/opt/msp430/lib/gcc-lib/msp430/3.2.3/include/stddef.h" 3
typedef int ptrdiff_t;
#line 213
typedef unsigned int size_t;
#line 325
typedef int wchar_t;
# 41 "/opt/msp430/msp430/include/sys/types.h"
typedef unsigned char u_char;
typedef unsigned short u_short;
typedef unsigned int u_int;
typedef unsigned long u_long;
typedef unsigned short ushort;
typedef unsigned int uint;

typedef uint8_t u_int8_t;
typedef uint16_t u_int16_t;
typedef uint32_t u_int32_t;
typedef uint64_t u_int64_t;

typedef u_int64_t u_quad_t;
typedef int64_t quad_t;
typedef quad_t *qaddr_t;

typedef char *caddr_t;
typedef const char *c_caddr_t;
typedef volatile char *v_caddr_t;
typedef u_int32_t fixpt_t;
typedef u_int32_t gid_t;
typedef u_int32_t in_addr_t;
typedef u_int16_t in_port_t;
typedef u_int32_t ino_t;
typedef long key_t;
typedef u_int16_t mode_t;
typedef u_int16_t nlink_t;
typedef quad_t rlim_t;
typedef int32_t segsz_t;
typedef int32_t swblk_t;
typedef int32_t ufs_daddr_t;
typedef int32_t ufs_time_t;
typedef u_int32_t uid_t;
# 40 "/opt/msp430/msp430/include/string.h"
extern void *memset(void *, int , size_t );
#line 61
extern void *memset(void *, int , size_t );
# 59 "/opt/msp430/msp430/include/stdlib.h"
#line 55
typedef struct __nesc_unnamed4242 {

  int quot;
  int rem;
} div_t;







#line 63
typedef struct __nesc_unnamed4243 {

  long quot;
  long rem;
} ldiv_t;
# 122 "/opt/msp430/msp430/include/sys/config.h" 3
typedef long int __int32_t;
typedef unsigned long int __uint32_t;
# 12 "/opt/msp430/msp430/include/sys/_types.h"
typedef long _off_t;
typedef long _ssize_t;
# 28 "/opt/msp430/msp430/include/sys/reent.h" 3
typedef __uint32_t __ULong;


struct _glue {

  struct _glue *_next;
  int _niobs;
  struct __sFILE *_iobs;
};

struct _Bigint {

  struct _Bigint *_next;
  int _k, _maxwds, _sign, _wds;
  __ULong _x[1];
};


struct __tm {

  int __tm_sec;
  int __tm_min;
  int __tm_hour;
  int __tm_mday;
  int __tm_mon;
  int __tm_year;
  int __tm_wday;
  int __tm_yday;
  int __tm_isdst;
};







struct _atexit {
  struct _atexit *_next;
  int _ind;
  void (*_fns[32])(void );
};








struct __sbuf {
  unsigned char *_base;
  int _size;
};






typedef long _fpos_t;
#line 116
struct __sFILE {
  unsigned char *_p;
  int _r;
  int _w;
  short _flags;
  short _file;
  struct __sbuf _bf;
  int _lbfsize;


  void *_cookie;

  int (*_read)(void *_cookie, char *_buf, int _n);
  int (*_write)(void *_cookie, const char *_buf, int _n);

  _fpos_t (*_seek)(void *_cookie, _fpos_t _offset, int _whence);
  int (*_close)(void *_cookie);


  struct __sbuf _ub;
  unsigned char *_up;
  int _ur;


  unsigned char _ubuf[3];
  unsigned char _nbuf[1];


  struct __sbuf _lb;


  int _blksize;
  int _offset;

  struct _reent *_data;
};
#line 174
struct _rand48 {
  unsigned short _seed[3];
  unsigned short _mult[3];
  unsigned short _add;
};









struct _reent {


  int _errno;




  struct __sFILE *_stdin, *_stdout, *_stderr;

  int _inc;
  char _emergency[25];

  int _current_category;
  const char *_current_locale;

  int __sdidinit;

  void (*__cleanup)(struct _reent *);


  struct _Bigint *_result;
  int _result_k;
  struct _Bigint *_p5s;
  struct _Bigint **_freelist;


  int _cvtlen;
  char *_cvtbuf;

  union __nesc_unnamed4244 {

    struct __nesc_unnamed4245 {

      unsigned int _unused_rand;
      char *_strtok_last;
      char _asctime_buf[26];
      struct __tm _localtime_buf;
      int _gamma_signgam;
      __extension__ unsigned long long _rand_next;
      struct _rand48 _r48;
    } _reent;



    struct __nesc_unnamed4246 {


      unsigned char *_nextf[30];
      unsigned int _nmalloc[30];
    } _unused;
  } _new;


  struct _atexit *_atexit;
  struct _atexit _atexit0;


  void (**_sig_func)(int );




  struct _glue __sglue;
  struct __sFILE __sf[3];
};
#line 273
struct _reent;
# 18 "/opt/msp430/msp430/include/math.h"
union __dmath {

  __uint32_t i[2];
  double d;
};




union __dmath;
#line 208
struct exception {


  int type;
  char *name;
  double arg1;
  double arg2;
  double retval;
  int err;
};
#line 261
enum __fdlibm_version {

  __fdlibm_ieee = -1, 
  __fdlibm_svid, 
  __fdlibm_xopen, 
  __fdlibm_posix
};




enum __fdlibm_version;
# 20 "/opt/tinyos-2.x/tos/system/tos.h"
typedef uint8_t bool;
enum __nesc_unnamed4247 {
#line 21
  FALSE = 0, TRUE = 1
};
typedef nx_int8_t nx_bool;






struct __nesc_attr_atmostonce {
};
#line 31
struct __nesc_attr_atleastonce {
};
#line 32
struct __nesc_attr_exactlyonce {
};
# 34 "/opt/tinyos-2.x/tos/types/TinyError.h"
enum __nesc_unnamed4248 {
  SUCCESS = 0, 
  FAIL = 1, 
  ESIZE = 2, 
  ECANCEL = 3, 
  EOFF = 4, 
  EBUSY = 5, 
  EINVAL = 6, 
  ERETRY = 7, 
  ERESERVE = 8, 
  EALREADY = 9
};

typedef uint8_t error_t  ;

static inline error_t ecombine(error_t r1, error_t r2);
# 39 "/opt/msp430/msp430/include/msp430/iostructures.h"
#line 27
typedef union port {
  volatile unsigned char reg_p;
  volatile struct __nesc_unnamed4249 {
    unsigned char __p0 : 1, 
    __p1 : 1, 
    __p2 : 1, 
    __p3 : 1, 
    __p4 : 1, 
    __p5 : 1, 
    __p6 : 1, 
    __p7 : 1;
  } __pin;
} __attribute((packed))  ioregister_t;
# 108 "/opt/msp430/msp430/include/msp430/iostructures.h" 3
struct port_full_t {
  ioregister_t in;
  ioregister_t out;
  ioregister_t dir;
  ioregister_t ifg;
  ioregister_t ies;
  ioregister_t ie;
  ioregister_t sel;
};









struct port_simple_t {
  ioregister_t in;
  ioregister_t out;
  ioregister_t dir;
  ioregister_t sel;
};




struct port_full_t;



struct port_full_t;



struct port_simple_t;



struct port_simple_t;



struct port_simple_t;



struct port_simple_t;
# 116 "/opt/msp430/msp430/include/msp430/gpio.h" 3
volatile unsigned char P1OUT __asm ("0x0021");

volatile unsigned char P1DIR __asm ("0x0022");





volatile unsigned char P1IE __asm ("0x0025");

volatile unsigned char P1SEL __asm ("0x0026");










volatile unsigned char P2OUT __asm ("0x0029");

volatile unsigned char P2DIR __asm ("0x002A");





volatile unsigned char P2IE __asm ("0x002D");

volatile unsigned char P2SEL __asm ("0x002E");










volatile unsigned char P3OUT __asm ("0x0019");

volatile unsigned char P3DIR __asm ("0x001A");

volatile unsigned char P3SEL __asm ("0x001B");










volatile unsigned char P4OUT __asm ("0x001D");

volatile unsigned char P4DIR __asm ("0x001E");

volatile unsigned char P4SEL __asm ("0x001F");










volatile unsigned char P5OUT __asm ("0x0031");

volatile unsigned char P5DIR __asm ("0x0032");

volatile unsigned char P5SEL __asm ("0x0033");










volatile unsigned char P6OUT __asm ("0x0035");

volatile unsigned char P6DIR __asm ("0x0036");

volatile unsigned char P6SEL __asm ("0x0037");
# 99 "/opt/msp430/msp430/include/msp430/usart.h"
volatile unsigned char U0TCTL __asm ("0x0071");
#line 283
volatile unsigned char U1TCTL __asm ("0x0079");
# 27 "/opt/msp430/msp430/include/msp430/timera.h"
volatile unsigned int TA0CTL __asm ("0x0160");

volatile unsigned int TA0R __asm ("0x0170");


volatile unsigned int TA0CCTL0 __asm ("0x0162");

volatile unsigned int TA0CCTL1 __asm ("0x0164");
#line 70
volatile unsigned int TA0CCTL2 __asm ("0x0166");
# 127 "/opt/msp430/msp430/include/msp430/timera.h" 3
#line 118
typedef struct __nesc_unnamed4250 {
  volatile unsigned 
  taifg : 1, 
  taie : 1, 
  taclr : 1, 
  dummy : 1, 
  tamc : 2, 
  taid : 2, 
  tassel : 2;
} __attribute((packed))  tactl_t;
#line 143
#line 129
typedef struct __nesc_unnamed4251 {
  volatile unsigned 
  ccifg : 1, 
  cov : 1, 
  out : 1, 
  cci : 1, 
  ccie : 1, 
  outmod : 3, 
  cap : 1, 
  dummy : 1, 
  scci : 1, 
  scs : 1, 
  ccis : 2, 
  cm : 2;
} __attribute((packed))  tacctl_t;


struct timera_t {
  tactl_t ctl;
  tacctl_t cctl0;
  tacctl_t cctl1;
  tacctl_t cctl2;
  volatile unsigned dummy[4];
  volatile unsigned tar;
  volatile unsigned taccr0;
  volatile unsigned taccr1;
  volatile unsigned taccr2;
};



struct timera_t;
# 26 "/opt/msp430/msp430/include/msp430/timerb.h"
volatile unsigned int TBR __asm ("0x0190");


volatile unsigned int TBCCTL0 __asm ("0x0182");





volatile unsigned int TBCCR0 __asm ("0x0192");
#line 76
#line 64
typedef struct __nesc_unnamed4252 {
  volatile unsigned 
  tbifg : 1, 
  tbie : 1, 
  tbclr : 1, 
  dummy1 : 1, 
  tbmc : 2, 
  tbid : 2, 
  tbssel : 2, 
  dummy2 : 1, 
  tbcntl : 2, 
  tbclgrp : 2;
} __attribute((packed))  tbctl_t;
#line 91
#line 78
typedef struct __nesc_unnamed4253 {
  volatile unsigned 
  ccifg : 1, 
  cov : 1, 
  out : 1, 
  cci : 1, 
  ccie : 1, 
  outmod : 3, 
  cap : 1, 
  clld : 2, 
  scs : 1, 
  ccis : 2, 
  cm : 2;
} __attribute((packed))  tbcctl_t;


struct timerb_t {
  tbctl_t ctl;
  tbcctl_t cctl0;
  tbcctl_t cctl1;
  tbcctl_t cctl2;

  tbcctl_t cctl3;
  tbcctl_t cctl4;
  tbcctl_t cctl5;
  tbcctl_t cctl6;



  volatile unsigned tbr;
  volatile unsigned tbccr0;
  volatile unsigned tbccr1;
  volatile unsigned tbccr2;

  volatile unsigned tbccr3;
  volatile unsigned tbccr4;
  volatile unsigned tbccr5;
  volatile unsigned tbccr6;
};





struct timerb_t;
# 20 "/opt/msp430/msp430/include/msp430/basic_clock.h"
volatile unsigned char DCOCTL __asm ("0x0056");

volatile unsigned char BCSCTL1 __asm ("0x0057");

volatile unsigned char BCSCTL2 __asm ("0x0058");
# 18 "/opt/msp430/msp430/include/msp430/adc12.h"
volatile unsigned int ADC12CTL0 __asm ("0x01A0");

volatile unsigned int ADC12CTL1 __asm ("0x01A2");





volatile unsigned int ADC12IV __asm ("0x01A8");
#line 42
#line 30
typedef struct __nesc_unnamed4254 {
  volatile unsigned 
  adc12sc : 1, 
  enc : 1, 
  adc12tovie : 1, 
  adc12ovie : 1, 
  adc12on : 1, 
  refon : 1, 
  r2_5v : 1, 
  msc : 1, 
  sht0 : 4, 
  sht1 : 4;
} __attribute((packed))  adc12ctl0_t;
#line 54
#line 44
typedef struct __nesc_unnamed4255 {
  volatile unsigned 
  adc12busy : 1, 
  conseq : 2, 
  adc12ssel : 2, 
  adc12div : 3, 
  issh : 1, 
  shp : 1, 
  shs : 2, 
  cstartadd : 4;
} __attribute((packed))  adc12ctl1_t;
#line 74
#line 56
typedef struct __nesc_unnamed4256 {
  volatile unsigned 
  bit0 : 1, 
  bit1 : 1, 
  bit2 : 1, 
  bit3 : 1, 
  bit4 : 1, 
  bit5 : 1, 
  bit6 : 1, 
  bit7 : 1, 
  bit8 : 1, 
  bit9 : 1, 
  bit10 : 1, 
  bit11 : 1, 
  bit12 : 1, 
  bit13 : 1, 
  bit14 : 1, 
  bit15 : 1;
} __attribute((packed))  adc12xflg_t;


struct adc12_t {
  adc12ctl0_t ctl0;
  adc12ctl1_t ctl1;
  adc12xflg_t ifg;
  adc12xflg_t ie;
  adc12xflg_t iv;
};




struct adc12_t;
# 83 "/opt/msp430/msp430/include/msp430x16x.h"
volatile unsigned char ME1 __asm ("0x0004");





volatile unsigned char ME2 __asm ("0x0005");
# 177 "/opt/tinyos-2.x/tos/chips/msp430/msp430hardware.h"
typedef uint8_t mcu_power_t  ;
static inline mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2);


enum __nesc_unnamed4257 {
  MSP430_POWER_ACTIVE = 0, 
  MSP430_POWER_LPM0 = 1, 
  MSP430_POWER_LPM1 = 2, 
  MSP430_POWER_LPM2 = 3, 
  MSP430_POWER_LPM3 = 4, 
  MSP430_POWER_LPM4 = 5
};

static inline void __nesc_disable_interrupt(void );





static inline void __nesc_enable_interrupt(void );




typedef bool __nesc_atomic_t;
__nesc_atomic_t __nesc_atomic_start(void );
void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts);






__nesc_atomic_t __nesc_atomic_start(void )  ;






void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)  ;
# 31 "/opt/tinyos-2.x/tos/platforms/hmote2420/hardware.h"
static inline void TOSH_SET_SIMO0_PIN(void);
#line 31
static inline void TOSH_CLR_SIMO0_PIN(void);
#line 31
static inline void TOSH_MAKE_SIMO0_OUTPUT(void);
#line 31
static inline void TOSH_MAKE_SIMO0_INPUT(void);
static inline void TOSH_SET_UCLK0_PIN(void);
#line 32
static inline void TOSH_CLR_UCLK0_PIN(void);
#line 32
static inline void TOSH_MAKE_UCLK0_OUTPUT(void);
#line 32
static inline void TOSH_MAKE_UCLK0_INPUT(void);
#line 69
enum __nesc_unnamed4258 {

  TOSH_HUMIDITY_ADDR = 5, 
  TOSH_HUMIDTEMP_ADDR = 3, 
  TOSH_HUMIDITY_RESET = 0x1E
};



static inline void TOSH_SET_FLASH_CS_PIN(void);
#line 78
static inline void TOSH_CLR_FLASH_CS_PIN(void);
#line 78
static inline void TOSH_MAKE_FLASH_CS_OUTPUT(void);
static inline void TOSH_SET_FLASH_HOLD_PIN(void);
#line 79
static inline void TOSH_CLR_FLASH_HOLD_PIN(void);
#line 79
static inline void TOSH_MAKE_FLASH_HOLD_OUTPUT(void);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.h"
enum __nesc_unnamed4259 {
  MSP430TIMER_CM_NONE = 0, 
  MSP430TIMER_CM_RISING = 1, 
  MSP430TIMER_CM_FALLING = 2, 
  MSP430TIMER_CM_BOTH = 3, 

  MSP430TIMER_STOP_MODE = 0, 
  MSP430TIMER_UP_MODE = 1, 
  MSP430TIMER_CONTINUOUS_MODE = 2, 
  MSP430TIMER_UPDOWN_MODE = 3, 

  MSP430TIMER_TACLK = 0, 
  MSP430TIMER_TBCLK = 0, 
  MSP430TIMER_ACLK = 1, 
  MSP430TIMER_SMCLK = 2, 
  MSP430TIMER_INCLK = 3, 

  MSP430TIMER_CLOCKDIV_1 = 0, 
  MSP430TIMER_CLOCKDIV_2 = 1, 
  MSP430TIMER_CLOCKDIV_4 = 2, 
  MSP430TIMER_CLOCKDIV_8 = 3
};
#line 64
#line 51
typedef struct __nesc_unnamed4260 {

  int ccifg : 1;
  int cov : 1;
  int out : 1;
  int cci : 1;
  int ccie : 1;
  int outmod : 3;
  int cap : 1;
  int clld : 2;
  int scs : 1;
  int ccis : 2;
  int cm : 2;
} msp430_compare_control_t;
#line 76
#line 66
typedef struct __nesc_unnamed4261 {

  int taifg : 1;
  int taie : 1;
  int taclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tassel : 2;
  int _unused1 : 6;
} msp430_timer_a_control_t;
#line 91
#line 78
typedef struct __nesc_unnamed4262 {

  int tbifg : 1;
  int tbie : 1;
  int tbclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tbssel : 2;
  int _unused1 : 1;
  int cntl : 2;
  int tbclgrp : 2;
  int _unused2 : 1;
} msp430_timer_b_control_t;
# 30 "/opt/tinyos-2.x/tos/types/Leds.h"
enum __nesc_unnamed4263 {
  LEDS_LED0 = 1 << 0, 
  LEDS_LED1 = 1 << 1, 
  LEDS_LED2 = 1 << 2, 
  LEDS_LED3 = 1 << 3, 
  LEDS_LED4 = 1 << 4, 
  LEDS_LED5 = 1 << 5, 
  LEDS_LED6 = 1 << 6, 
  LEDS_LED7 = 1 << 7
};
# 29 "/opt/tinyos-2.x/tos/lib/timer/Timer.h"
typedef struct __nesc_unnamed4264 {
} 
#line 29
TMilli;
typedef struct __nesc_unnamed4265 {
} 
#line 30
T32khz;
typedef struct __nesc_unnamed4266 {
} 
#line 31
TMicro;
# 55 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12.h"
#line 44
typedef struct __nesc_unnamed4267 {

  unsigned int inch : 4;
  unsigned int sref : 3;
  unsigned int ref2_5v : 1;
  unsigned int adc12ssel : 2;
  unsigned int adc12div : 3;
  unsigned int sht : 4;
  unsigned int sampcon_ssel : 2;
  unsigned int sampcon_id : 2;
  unsigned int  : 0;
} msp430adc12_channel_config_t;








#line 57
typedef struct __nesc_unnamed4268 {


  volatile unsigned 
  inch : 4, 
  sref : 3, 
  eos : 1;
} __attribute((packed))  adc12memctl_t;

enum inch_enum {


  INPUT_CHANNEL_A0 = 0, 
  INPUT_CHANNEL_A1 = 1, 
  INPUT_CHANNEL_A2 = 2, 
  INPUT_CHANNEL_A3 = 3, 
  INPUT_CHANNEL_A4 = 4, 
  INPUT_CHANNEL_A5 = 5, 
  INPUT_CHANNEL_A6 = 6, 
  INPUT_CHANNEL_A7 = 7, 
  EXTERNAL_REF_VOLTAGE_CHANNEL = 8, 
  REF_VOLTAGE_NEG_TERMINAL_CHANNEL = 9, 
  TEMPERATURE_DIODE_CHANNEL = 10, 
  SUPPLY_VOLTAGE_HALF_CHANNEL = 11, 
  INPUT_CHANNEL_NONE = 12
};

enum sref_enum {

  REFERENCE_AVcc_AVss = 0, 
  REFERENCE_VREFplus_AVss = 1, 
  REFERENCE_VeREFplus_AVss = 2, 
  REFERENCE_AVcc_VREFnegterm = 4, 
  REFERENCE_VREFplus_VREFnegterm = 5, 
  REFERENCE_VeREFplus_VREFnegterm = 6
};

enum ref2_5v_enum {

  REFVOLT_LEVEL_1_5 = 0, 
  REFVOLT_LEVEL_2_5 = 1, 
  REFVOLT_LEVEL_NONE = 0
};

enum adc12ssel_enum {

  SHT_SOURCE_ADC12OSC = 0, 
  SHT_SOURCE_ACLK = 1, 
  SHT_SOURCE_MCLK = 2, 
  SHT_SOURCE_SMCLK = 3
};

enum adc12div_enum {

  SHT_CLOCK_DIV_1 = 0, 
  SHT_CLOCK_DIV_2 = 1, 
  SHT_CLOCK_DIV_3 = 2, 
  SHT_CLOCK_DIV_4 = 3, 
  SHT_CLOCK_DIV_5 = 4, 
  SHT_CLOCK_DIV_6 = 5, 
  SHT_CLOCK_DIV_7 = 6, 
  SHT_CLOCK_DIV_8 = 7
};

enum sht_enum {

  SAMPLE_HOLD_4_CYCLES = 0, 
  SAMPLE_HOLD_8_CYCLES = 1, 
  SAMPLE_HOLD_16_CYCLES = 2, 
  SAMPLE_HOLD_32_CYCLES = 3, 
  SAMPLE_HOLD_64_CYCLES = 4, 
  SAMPLE_HOLD_96_CYCLES = 5, 
  SAMPLE_HOLD_123_CYCLES = 6, 
  SAMPLE_HOLD_192_CYCLES = 7, 
  SAMPLE_HOLD_256_CYCLES = 8, 
  SAMPLE_HOLD_384_CYCLES = 9, 
  SAMPLE_HOLD_512_CYCLES = 10, 
  SAMPLE_HOLD_768_CYCLES = 11, 
  SAMPLE_HOLD_1024_CYCLES = 12
};

enum sampcon_ssel_enum {

  SAMPCON_SOURCE_TACLK = 0, 
  SAMPCON_SOURCE_ACLK = 1, 
  SAMPCON_SOURCE_SMCLK = 2, 
  SAMPCON_SOURCE_INCLK = 3
};

enum sampcon_id_enum {

  SAMPCON_CLOCK_DIV_1 = 0, 
  SAMPCON_CLOCK_DIV_2 = 1, 
  SAMPCON_CLOCK_DIV_3 = 2, 
  SAMPCON_CLOCK_DIV_4 = 3
};
# 33 "/opt/tinyos-2.x/tos/types/Resource.h"
typedef uint8_t resource_client_id_t;
typedef uint16_t LightSensorM$Read$val_t;
typedef TMilli LightSensorM$T1$precision_tag;
typedef TMilli LightSensorM$T2$precision_tag;
typedef const msp430adc12_channel_config_t *AdcP$ConfigReadStream$adc_config_t;
typedef uint16_t AdcP$Read$val_t;
typedef uint16_t AdcP$ReadNow$val_t;
typedef const msp430adc12_channel_config_t *AdcP$Config$adc_config_t;
typedef uint16_t AdcP$ReadStream$val_t;
typedef TMilli Msp430RefVoltGeneratorP$SwitchOffTimer$precision_tag;
typedef TMilli Msp430RefVoltGeneratorP$SwitchOnTimer$precision_tag;
typedef const msp430adc12_channel_config_t *Msp430RefVoltArbiterImplP$Config$adc_config_t;
enum /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Timer*/Msp430Timer32khzC$0$__nesc_unnamed4269 {
  Msp430Timer32khzC$0$ALARM_ID = 0U
};
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$frequency_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$frequency_tag /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$size_type;
typedef T32khz /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$frequency_tag;
typedef /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$frequency_tag /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$precision_tag;
typedef uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$size_type;
typedef TMilli /*CounterMilli32C.Transform*/TransformCounterC$0$to_precision_tag;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type;
typedef T32khz /*CounterMilli32C.Transform*/TransformCounterC$0$from_precision_tag;
typedef uint16_t /*CounterMilli32C.Transform*/TransformCounterC$0$from_size_type;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC$0$upper_count_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC$0$from_precision_tag /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC$0$from_size_type /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$size_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC$0$to_precision_tag /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$size_type;
typedef TMilli /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type;
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$from_precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$from_size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$from_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$from_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$size_type;
typedef TMilli /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$precision_tag;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$precision_tag;
typedef TMilli /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$precision_tag;
enum /*LightSensorC.Sensor.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$__nesc_unnamed4270 {
  Msp430Adc12ClientAutoRVGC$0$ID = 0U
};
enum /*LightSensorC.Sensor.AdcReadClientC*/AdcReadClientC$0$__nesc_unnamed4271 {
  AdcReadClientC$0$CLIENT = 0U
};
enum /*LightSensorC.Sensor.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$1$__nesc_unnamed4272 {
  Msp430Adc12ClientAutoRVGC$1$ID = 1U
};
enum /*LightSensorC.Sensor.AdcReadStreamClientC*/AdcReadStreamClientC$0$__nesc_unnamed4273 {
  AdcReadStreamClientC$0$RSCLIENT = 0U
};
typedef const msp430adc12_channel_config_t *LightP$AdcConfigure$adc_config_t;
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static  error_t PlatformP$Init$init(void);
#line 51
static  error_t MotePlatformC$Init$init(void);
# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
static  void Msp430ClockP$Msp430ClockInit$default$initTimerB(void);


static  void Msp430ClockP$Msp430ClockInit$defaultInitTimerA(void);
#line 29
static  void Msp430ClockP$Msp430ClockInit$default$initTimerA(void);




static  void Msp430ClockP$Msp430ClockInit$defaultInitTimerB(void);
#line 28
static  void Msp430ClockP$Msp430ClockInit$default$initClocks(void);



static  void Msp430ClockP$Msp430ClockInit$defaultInitClocks(void);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static  error_t Msp430ClockP$Init$init(void);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX0$fired(void);
#line 28
static   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Overflow$fired(void);
#line 28
static   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX1$fired(void);
#line 28
static   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$default$fired(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x12aa9a8);
# 41 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$clear(void);


static   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setClockSource(uint16_t arg_0x1279178);
#line 43
static   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$disableEvents(void);
#line 39
static   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setMode(int arg_0x127b088);





static   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setInputDivider(uint16_t arg_0x1279620);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static   void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX0$fired(void);
#line 28
static   void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Overflow$fired(void);
#line 28
static   void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX1$fired(void);
#line 28
static   void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$default$fired(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x12aa9a8);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$get(void);
static   bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$isOverflowPending(void);
# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$getEvent(void);
#line 75
static   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$default$captured(uint16_t arg_0x1294d38);
# 31 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static   msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Control$getControl(void);



static   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Control$setControl(msp430_compare_control_t arg_0x12848c0);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Event$fired(void);
# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$setEvent(uint16_t arg_0x128c8e0);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Timer$overflow(void);
# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$getEvent(void);
#line 75
static   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$default$captured(uint16_t arg_0x1294d38);
# 31 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static   msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Control$getControl(void);



static   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Control$setControl(msp430_compare_control_t arg_0x12848c0);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Event$fired(void);
# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$setEvent(uint16_t arg_0x128c8e0);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Timer$overflow(void);
# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$getEvent(void);
#line 75
static   void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$default$captured(uint16_t arg_0x1294d38);
# 31 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static   msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Control$getControl(void);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static   void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Event$fired(void);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$default$fired(void);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Timer$overflow(void);
# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$getEvent(void);
#line 75
static   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$default$captured(uint16_t arg_0x1294d38);
# 31 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static   msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$getControl(void);







static   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$enableEvents(void);
#line 36
static   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$setControlAsCompare(void);



static   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$disableEvents(void);
#line 33
static   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$clearPendingInterrupt(void);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Event$fired(void);
# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEvent(uint16_t arg_0x128c8e0);

static   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEventFromNow(uint16_t arg_0x128b248);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$overflow(void);
# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$getEvent(void);
#line 75
static   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$default$captured(uint16_t arg_0x1294d38);
# 31 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static   msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$getControl(void);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Event$fired(void);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$default$fired(void);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Timer$overflow(void);
# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$getEvent(void);
#line 75
static   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$default$captured(uint16_t arg_0x1294d38);
# 31 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static   msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$getControl(void);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Event$fired(void);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$default$fired(void);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Timer$overflow(void);
# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$getEvent(void);
#line 75
static   void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$default$captured(uint16_t arg_0x1294d38);
# 31 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static   msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Control$getControl(void);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static   void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Event$fired(void);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$default$fired(void);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Timer$overflow(void);
# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$getEvent(void);
#line 75
static   void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$default$captured(uint16_t arg_0x1294d38);
# 31 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static   msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Control$getControl(void);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static   void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Event$fired(void);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$default$fired(void);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Timer$overflow(void);
# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$getEvent(void);
#line 75
static   void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$default$captured(uint16_t arg_0x1294d38);
# 31 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static   msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Control$getControl(void);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static   void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Event$fired(void);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$default$fired(void);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Timer$overflow(void);
# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$getEvent(void);
#line 75
static   void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$default$captured(uint16_t arg_0x1294d38);
# 31 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static   msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Control$getControl(void);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static   void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Event$fired(void);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$default$fired(void);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Timer$overflow(void);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static   error_t SchedulerBasicP$TaskBasic$postTask(
# 45 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x11e29a8);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static  void SchedulerBasicP$TaskBasic$default$runTask(
# 45 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x11e29a8);
# 46 "/opt/tinyos-2.x/tos/interfaces/Scheduler.nc"
static  void SchedulerBasicP$Scheduler$init(void);
#line 61
static  void SchedulerBasicP$Scheduler$taskLoop(void);
#line 54
static  bool SchedulerBasicP$Scheduler$runNextTask(void);
# 54 "/opt/tinyos-2.x/tos/interfaces/McuPowerOverride.nc"
static   mcu_power_t McuSleepC$McuPowerOverride$default$lowestState(void);
# 59 "/opt/tinyos-2.x/tos/interfaces/McuSleep.nc"
static   void McuSleepC$McuSleep$sleep(void);
# 49 "/opt/tinyos-2.x/tos/interfaces/Boot.nc"
static  void LightSensorM$Boot$booted(void);
# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static  void LightSensorM$Read$readDone(error_t arg_0x13b47f0, LightSensorM$Read$val_t arg_0x13b4970);
# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static  void LightSensorM$T1$fired(void);
#line 72
static  void LightSensorM$T2$fired(void);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static  error_t LedsP$Init$init(void);
# 50 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
static   void LedsP$Leds$led0Off(void);





static   void LedsP$Leds$led0Toggle(void);




static   void LedsP$Leds$led1On(void);




static   void LedsP$Leds$led1Off(void);
#line 83
static   void LedsP$Leds$led2Off(void);
#line 123
static   void LedsP$Leds$set(uint8_t arg_0x1395630);
#line 45
static   void LedsP$Leds$led0On(void);
#line 78
static   void LedsP$Leds$led2On(void);
# 44 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static   void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$toggle(void);
#line 71
static   void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$makeOutput(void);
#line 34
static   void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$set(void);




static   void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$clr(void);
#line 71
static   void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$makeOutput(void);
#line 34
static   void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$set(void);




static   void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$clr(void);
#line 71
static   void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$makeOutput(void);
#line 34
static   void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$set(void);




static   void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$clr(void);
#line 64
static   void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$makeInput(void);
#line 85
static   void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$selectIOFunc(void);
#line 78
static   void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$selectModuleFunc(void);
#line 64
static   void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$makeInput(void);
#line 85
static   void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$selectIOFunc(void);
#line 78
static   void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$selectModuleFunc(void);
#line 64
static   void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$makeInput(void);
#line 85
static   void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$selectIOFunc(void);
#line 78
static   void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$selectModuleFunc(void);
#line 64
static   void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$makeInput(void);
#line 85
static   void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$selectIOFunc(void);
#line 78
static   void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$selectModuleFunc(void);
#line 64
static   void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$makeInput(void);
#line 85
static   void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$selectIOFunc(void);
#line 78
static   void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$selectModuleFunc(void);
#line 64
static   void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$makeInput(void);
#line 85
static   void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$selectIOFunc(void);
#line 78
static   void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$selectModuleFunc(void);
#line 64
static   void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$makeInput(void);
#line 85
static   void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$selectIOFunc(void);
#line 78
static   void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$selectModuleFunc(void);
#line 64
static   void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$makeInput(void);
#line 85
static   void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$selectIOFunc(void);
#line 78
static   void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$selectModuleFunc(void);
# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$toggle(void);



static   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$makeOutput(void);
#line 29
static   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$set(void);
static   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$clr(void);




static   void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$makeOutput(void);
#line 29
static   void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$set(void);
static   void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$clr(void);




static   void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$makeOutput(void);
#line 29
static   void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$set(void);
static   void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$clr(void);
# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
static   AdcP$ConfigReadStream$adc_config_t AdcP$ConfigReadStream$default$getConfiguration(
# 52 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15d7330);
# 189 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static   error_t AdcP$SingleChannelReadStream$default$getData(
# 53 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15d7ce8);
# 227 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static   uint16_t *AdcP$SingleChannelReadStream$multipleDataReady(
# 53 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15d7ce8, 
# 227 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t arg_0x15eb9d0[], uint16_t arg_0x15ebb60);
#line 138
static   error_t AdcP$SingleChannelReadStream$default$configureMultiple(
# 53 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15d7ce8, 
# 138 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t *arg_0x15d2840, uint16_t arg_0x15d29e8[], uint16_t arg_0x15d2b78, uint16_t arg_0x15d2d00);
#line 206
static   error_t AdcP$SingleChannelReadStream$singleDataReady(
# 53 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15d7ce8, 
# 206 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t arg_0x15eb230);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t AdcP$ResourceReadStream$default$release(
# 54 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15d6848);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void AdcP$ResourceReadStream$granted(
# 54 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15d6848);
# 55 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static  error_t AdcP$Read$read(
# 38 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15b2010);
# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static  void AdcP$Read$default$readDone(
# 38 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15b2010, 
# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
error_t arg_0x13b47f0, AdcP$Read$val_t arg_0x13b4970);
# 65 "/opt/tinyos-2.x/tos/interfaces/ReadNow.nc"
static   void AdcP$ReadNow$default$readDone(
# 39 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15b0010, 
# 65 "/opt/tinyos-2.x/tos/interfaces/ReadNow.nc"
error_t arg_0x15c87c0, AdcP$ReadNow$val_t arg_0x15c8940);
# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
static   AdcP$Config$adc_config_t AdcP$Config$default$getConfiguration(
# 49 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15dba28);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static  void AdcP$finishStreamRequest$runTask(void);
# 189 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static   error_t AdcP$SingleChannel$default$getData(
# 50 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15ea740);
# 84 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static   error_t AdcP$SingleChannel$default$configureSingle(
# 50 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15ea740, 
# 84 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t *arg_0x15d35b8);
#line 227
static   uint16_t *AdcP$SingleChannel$multipleDataReady(
# 50 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15ea740, 
# 227 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t arg_0x15eb9d0[], uint16_t arg_0x15ebb60);
#line 206
static   error_t AdcP$SingleChannel$singleDataReady(
# 50 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15ea740, 
# 206 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t arg_0x15eb230);
# 89 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
static  void AdcP$ReadStream$default$bufferDone(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15c61a0, 
# 89 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
error_t arg_0x1595718, 
AdcP$ReadStream$val_t *arg_0x15958c8, uint16_t arg_0x1595a50);
#line 102
static  void AdcP$ReadStream$default$readDone(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15c61a0, 
# 102 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
error_t arg_0x15940b0, uint32_t arg_0x1594240);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t AdcP$ResourceRead$default$release(
# 45 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15c0660);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t AdcP$ResourceRead$default$request(
# 45 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15c0660);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void AdcP$ResourceRead$granted(
# 45 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15c0660);
# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   bool AdcP$ResourceRead$default$isOwner(
# 45 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15c0660);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static  void AdcP$signalBufferDone$runTask(void);
#line 64
static  void AdcP$readDone$runTask(void);
# 110 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc"
static   void Msp430Adc12ImplP$MultiChannel$default$dataReady(
# 42 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x1664010, 
# 110 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc"
uint16_t *arg_0x1658e80, uint16_t arg_0x1656030);
# 112 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
static   void Msp430Adc12ImplP$HplAdc12$conversionDone(uint16_t arg_0x1672010);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void Msp430Adc12ImplP$CompareA1$fired(void);
# 49 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
static   void Msp430Adc12ImplP$Overflow$default$memOverflow(
# 43 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x1664798);
# 54 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
static   void Msp430Adc12ImplP$Overflow$default$conversionTimeOverflow(
# 43 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x1664798);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static  error_t Msp430Adc12ImplP$Init$init(void);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void Msp430Adc12ImplP$TimerA$overflow(void);
# 189 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static   error_t Msp430Adc12ImplP$SingleChannel$getData(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x1666508);
# 84 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static   error_t Msp430Adc12ImplP$SingleChannel$configureSingle(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x1666508, 
# 84 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t *arg_0x15d35b8);
#line 227
static   uint16_t *Msp430Adc12ImplP$SingleChannel$default$multipleDataReady(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x1666508, 
# 227 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t arg_0x15eb9d0[], uint16_t arg_0x15ebb60);
#line 138
static   error_t Msp430Adc12ImplP$SingleChannel$configureMultiple(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x1666508, 
# 138 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t *arg_0x15d2840, uint16_t arg_0x15d29e8[], uint16_t arg_0x15d2b78, uint16_t arg_0x15d2d00);
#line 206
static   error_t Msp430Adc12ImplP$SingleChannel$default$singleDataReady(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x1666508, 
# 206 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t arg_0x15eb230);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void Msp430Adc12ImplP$CompareA0$fired(void);
# 63 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
static   adc12ctl0_t HplAdc12P$HplAdc12$getCtl0(void);
#line 82
static   adc12memctl_t HplAdc12P$HplAdc12$getMCtl(uint8_t arg_0x16747a0);
#line 106
static   void HplAdc12P$HplAdc12$resetIFGs(void);
#line 118
static   bool HplAdc12P$HplAdc12$isBusy(void);
#line 75
static   void HplAdc12P$HplAdc12$setMCtl(uint8_t arg_0x1674068, adc12memctl_t arg_0x16741f8);
#line 128
static   void HplAdc12P$HplAdc12$startConversion(void);
#line 51
static   void HplAdc12P$HplAdc12$setCtl0(adc12ctl0_t arg_0x1677e98);
#line 89
static   uint16_t HplAdc12P$HplAdc12$getMem(uint8_t arg_0x1674d40);





static   void HplAdc12P$HplAdc12$setIEFlags(uint16_t arg_0x1673350);
#line 123
static   void HplAdc12P$HplAdc12$stopConversion(void);









static   void HplAdc12P$HplAdc12$enableConversion(void);
#line 57
static   void HplAdc12P$HplAdc12$setCtl1(adc12ctl1_t arg_0x1676400);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static  error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$Init$init(void);
# 69 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
static   error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$enqueue(resource_client_id_t arg_0x1727358);
#line 43
static   bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$isEmpty(void);








static   bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$isEnqueued(resource_client_id_t arg_0x17288d0);







static   resource_client_id_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$dequeue(void);
# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
static   void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceRequested$default$requested(
# 52 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x1735168);
# 55 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static   void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$unconfigure(
# 56 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x1735c18);
# 49 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static   void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$configure(
# 56 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x1735c18);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$release(
# 51 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x1737830);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$request(
# 51 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x1737830);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$default$granted(
# 51 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x1737830);
# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   bool /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$isOwner(
# 51 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x1737830);
# 88 "/opt/tinyos-2.x/tos/interfaces/ArbiterInfo.nc"
static   uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ArbiterInfo$userId(void);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static  void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask$runTask(void);
# 112 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
static   void Msp430RefVoltGeneratorP$HplAdc12$conversionDone(uint16_t arg_0x1672010);
# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static  void Msp430RefVoltGeneratorP$SwitchOffTimer$fired(void);
# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static  error_t Msp430RefVoltGeneratorP$RefVolt_2_5V$start(void);
#line 83
static  error_t Msp430RefVoltGeneratorP$RefVolt_1_5V$start(void);
#line 109
static  error_t Msp430RefVoltGeneratorP$RefVolt_1_5V$stop(void);
# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static  void Msp430RefVoltGeneratorP$SwitchOnTimer$fired(void);
# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
static   Msp430RefVoltArbiterImplP$Config$adc_config_t Msp430RefVoltArbiterImplP$Config$default$getConfiguration(
# 43 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x1795e58);
# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static  void Msp430RefVoltArbiterImplP$RefVolt_2_5V$startDone(error_t arg_0x177c638);
#line 117
static  void Msp430RefVoltArbiterImplP$RefVolt_2_5V$stopDone(error_t arg_0x177b1e0);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t Msp430RefVoltArbiterImplP$AdcResource$default$release(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x1797888);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t Msp430RefVoltArbiterImplP$AdcResource$default$request(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x1797888);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void Msp430RefVoltArbiterImplP$AdcResource$granted(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x1797888);
# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   bool Msp430RefVoltArbiterImplP$AdcResource$default$isOwner(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x1797888);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t Msp430RefVoltArbiterImplP$ClientResource$release(
# 38 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x1798ee0);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t Msp430RefVoltArbiterImplP$ClientResource$request(
# 38 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x1798ee0);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void Msp430RefVoltArbiterImplP$ClientResource$default$granted(
# 38 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x1798ee0);
# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   bool Msp430RefVoltArbiterImplP$ClientResource$isOwner(
# 38 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x1798ee0);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static  void Msp430RefVoltArbiterImplP$switchOff$runTask(void);
# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static  void Msp430RefVoltArbiterImplP$RefVolt_1_5V$startDone(error_t arg_0x177c638);
#line 117
static  void Msp430RefVoltArbiterImplP$RefVolt_1_5V$stopDone(error_t arg_0x177b1e0);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$fired(void);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$overflow(void);
# 92 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$startAt(/*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$size_type arg_0x17d8aa8, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$size_type arg_0x17d8c30);
#line 62
static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$stop(void);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static  error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Init$init(void);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$overflow(void);
# 53 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static   /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$size_type /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$get(void);






static   bool /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$isOverflowPending(void);










static   void /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$overflow(void);
#line 53
static   /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$size_type /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$get(void);
# 98 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static   /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getNow(void);
#line 92
static   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$size_type arg_0x17d8aa8, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$size_type arg_0x17d8c30);
#line 105
static   /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getAlarm(void);
#line 62
static   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$stop(void);




static   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$fired(void);
# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$overflow(void);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static  void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$runTask(void);
# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static   void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$fired(void);
# 125 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static  uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$getNow(void);
#line 118
static  void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$startOneShotAt(uint32_t arg_0x13a6df8, uint32_t arg_0x13a5010);
#line 67
static  void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$stop(void);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$runTask(void);
# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$fired(void);
#line 72
static  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$default$fired(
# 37 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x190d3c0);
# 81 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static  bool /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$isRunning(
# 37 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x190d3c0);
# 53 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startPeriodic(
# 37 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x190d3c0, 
# 53 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
uint32_t arg_0x1390c10);








static  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShot(
# 37 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x190d3c0, 
# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
uint32_t arg_0x13a81e8);




static  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$stop(
# 37 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x190d3c0);
# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
static   LightP$AdcConfigure$adc_config_t LightP$AdcConfigure$getConfiguration(void);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static  error_t PlatformP$Msp430ClockInit$init(void);
#line 51
static  error_t PlatformP$MoteInit$init(void);
#line 51
static  error_t PlatformP$LedsInit$init(void);
# 10 "/opt/tinyos-2.x/tos/platforms/hmote2420/PlatformP.nc"
static inline  error_t PlatformP$Init$init(void);
# 6 "/opt/tinyos-2.x/tos/platforms/hmote2420/MotePlatformC.nc"
static __inline void MotePlatformC$uwait(uint16_t u);




static __inline void MotePlatformC$TOSH_wait(void);




static void MotePlatformC$TOSH_FLASH_M25P_DP_bit(bool set);










static inline void MotePlatformC$TOSH_FLASH_M25P_DP(void);
#line 59
static inline  error_t MotePlatformC$Init$init(void);
# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
static  void Msp430ClockP$Msp430ClockInit$initTimerB(void);
#line 29
static  void Msp430ClockP$Msp430ClockInit$initTimerA(void);
#line 28
static  void Msp430ClockP$Msp430ClockInit$initClocks(void);
# 38 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
 static volatile uint8_t Msp430ClockP$IE1 __asm ("0x0000");
 static volatile uint16_t Msp430ClockP$TA0CTL __asm ("0x0160");
 static volatile uint16_t Msp430ClockP$TA0IV __asm ("0x012E");
 static volatile uint16_t Msp430ClockP$TBCTL __asm ("0x0180");
 static volatile uint16_t Msp430ClockP$TBIV __asm ("0x011E");

enum Msp430ClockP$__nesc_unnamed4274 {

  Msp430ClockP$ACLK_CALIB_PERIOD = 8, 
  Msp430ClockP$ACLK_HZ = 32768U, 
  Msp430ClockP$TARGET_DCO_DELTA = 4096000 / Msp430ClockP$ACLK_HZ * Msp430ClockP$ACLK_CALIB_PERIOD
};

static inline  void Msp430ClockP$Msp430ClockInit$defaultInitClocks(void);
#line 72
static inline  void Msp430ClockP$Msp430ClockInit$defaultInitTimerA(void);
#line 87
static inline  void Msp430ClockP$Msp430ClockInit$defaultInitTimerB(void);
#line 102
static inline   void Msp430ClockP$Msp430ClockInit$default$initClocks(void);




static inline   void Msp430ClockP$Msp430ClockInit$default$initTimerA(void);




static inline   void Msp430ClockP$Msp430ClockInit$default$initTimerB(void);





static inline void Msp430ClockP$startTimerA(void);
#line 130
static inline void Msp430ClockP$startTimerB(void);
#line 142
static void Msp430ClockP$set_dco_calib(int calib);





static inline uint16_t Msp430ClockP$test_calib_busywait_delta(int calib);
#line 171
static inline void Msp430ClockP$busyCalibrateDco(void);
#line 204
static inline  error_t Msp430ClockP$Init$init(void);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$fired(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x12aa9a8);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$overflow(void);
# 80 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setMode(int mode);









static inline   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$clear(void);









static inline   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$disableEvents(void);




static inline   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setClockSource(uint16_t clockSource);




static inline   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setInputDivider(uint16_t inputDivider);




static inline   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX0$fired(void);




static inline   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX1$fired(void);





static inline   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Overflow$fired(void);








static inline    void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$default$fired(uint8_t n);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static   void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$fired(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x12aa9a8);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$overflow(void);
# 51 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static   uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$get(void);
#line 70
static inline   bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$isOverflowPending(void);
#line 115
static inline   void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX0$fired(void);




static inline   void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX1$fired(void);





static inline   void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Overflow$fired(void);








static    void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$default$fired(uint8_t n);
# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$captured(uint16_t arg_0x1294d38);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$fired(void);
# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$CC2int(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t x);
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$int2CC(uint16_t x);
#line 74
static inline   /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Control$getControl(void);
#line 89
static inline   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Control$setControl(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t x);
#line 139
static inline   uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$getEvent(void);




static inline   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$setEvent(uint16_t x);
#line 169
static   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Event$fired(void);







static inline    void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$default$captured(uint16_t n);







static inline   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Timer$overflow(void);
# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$captured(uint16_t arg_0x1294d38);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$fired(void);
# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$CC2int(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t x);
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$int2CC(uint16_t x);
#line 74
static inline   /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Control$getControl(void);
#line 89
static inline   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Control$setControl(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t x);
#line 139
static inline   uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$getEvent(void);




static inline   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$setEvent(uint16_t x);
#line 169
static   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Event$fired(void);







static inline    void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$default$captured(uint16_t n);







static inline   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Timer$overflow(void);
# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$captured(uint16_t arg_0x1294d38);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$fired(void);
# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$cc_t;


static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$int2CC(uint16_t x);
#line 74
static inline   /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Control$getControl(void);
#line 139
static inline   uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$getEvent(void);
#line 169
static   void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Event$fired(void);







static inline    void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$default$captured(uint16_t n);



static inline    void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$default$fired(void);



static inline   void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Timer$overflow(void);
# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$captured(uint16_t arg_0x1294d38);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$fired(void);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$get(void);
# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t x);
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$int2CC(uint16_t x);

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$compareControl(void);
#line 74
static inline   /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$getControl(void);









static inline   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$clearPendingInterrupt(void);









static inline   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$setControlAsCompare(void);
#line 119
static inline   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$enableEvents(void);




static inline   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$disableEvents(void);
#line 139
static inline   uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$getEvent(void);




static inline   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEvent(uint16_t x);









static inline   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEventFromNow(uint16_t x);
#line 169
static inline   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Event$fired(void);







static inline    void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$default$captured(uint16_t n);







static inline   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$overflow(void);
# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$captured(uint16_t arg_0x1294d38);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$fired(void);
# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$cc_t;


static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$int2CC(uint16_t x);
#line 74
static inline   /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$getControl(void);
#line 139
static inline   uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$getEvent(void);
#line 169
static inline   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Event$fired(void);







static inline    void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$default$captured(uint16_t n);



static inline    void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$default$fired(void);



static inline   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Timer$overflow(void);
# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$captured(uint16_t arg_0x1294d38);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$fired(void);
# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$cc_t;


static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$int2CC(uint16_t x);
#line 74
static inline   /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$getControl(void);
#line 139
static inline   uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$getEvent(void);
#line 169
static inline   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Event$fired(void);







static inline    void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$default$captured(uint16_t n);



static inline    void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$default$fired(void);



static inline   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Timer$overflow(void);
# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$captured(uint16_t arg_0x1294d38);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$fired(void);
# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$cc_t;


static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$int2CC(uint16_t x);
#line 74
static inline   /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Control$getControl(void);
#line 139
static inline   uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$getEvent(void);
#line 169
static inline   void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Event$fired(void);







static inline    void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$default$captured(uint16_t n);



static inline    void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$default$fired(void);



static inline   void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Timer$overflow(void);
# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$captured(uint16_t arg_0x1294d38);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$fired(void);
# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$cc_t;


static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$int2CC(uint16_t x);
#line 74
static inline   /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Control$getControl(void);
#line 139
static inline   uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$getEvent(void);
#line 169
static inline   void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Event$fired(void);







static inline    void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$default$captured(uint16_t n);



static inline    void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$default$fired(void);



static inline   void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Timer$overflow(void);
# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$captured(uint16_t arg_0x1294d38);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$fired(void);
# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$cc_t;


static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$int2CC(uint16_t x);
#line 74
static inline   /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Control$getControl(void);
#line 139
static inline   uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$getEvent(void);
#line 169
static inline   void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Event$fired(void);







static inline    void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$default$captured(uint16_t n);



static inline    void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$default$fired(void);



static inline   void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Timer$overflow(void);
# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static   void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$captured(uint16_t arg_0x1294d38);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$fired(void);
# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$cc_t;


static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$int2CC(uint16_t x);
#line 74
static inline   /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Control$getControl(void);
#line 139
static inline   uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$getEvent(void);
#line 169
static inline   void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Event$fired(void);







static inline    void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$default$captured(uint16_t n);



static inline    void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$default$fired(void);



static inline   void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Timer$overflow(void);
# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static   void Msp430TimerCommonP$VectorTimerB1$fired(void);
#line 28
static   void Msp430TimerCommonP$VectorTimerA0$fired(void);
#line 28
static   void Msp430TimerCommonP$VectorTimerA1$fired(void);
#line 28
static   void Msp430TimerCommonP$VectorTimerB0$fired(void);
# 11 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
void sig_TIMERA0_VECTOR(void)  __attribute((wakeup)) __attribute((interrupt(12))) ;
void sig_TIMERA1_VECTOR(void)  __attribute((wakeup)) __attribute((interrupt(10))) ;
void sig_TIMERB0_VECTOR(void)  __attribute((wakeup)) __attribute((interrupt(26))) ;
void sig_TIMERB1_VECTOR(void)  __attribute((wakeup)) __attribute((interrupt(24))) ;
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static  error_t RealMainP$SoftwareInit$init(void);
# 49 "/opt/tinyos-2.x/tos/interfaces/Boot.nc"
static  void RealMainP$Boot$booted(void);
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static  error_t RealMainP$PlatformInit$init(void);
# 46 "/opt/tinyos-2.x/tos/interfaces/Scheduler.nc"
static  void RealMainP$Scheduler$init(void);
#line 61
static  void RealMainP$Scheduler$taskLoop(void);
#line 54
static  bool RealMainP$Scheduler$runNextTask(void);
# 52 "/opt/tinyos-2.x/tos/system/RealMainP.nc"
int main(void)   ;
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static  void SchedulerBasicP$TaskBasic$runTask(
# 45 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x11e29a8);
# 59 "/opt/tinyos-2.x/tos/interfaces/McuSleep.nc"
static   void SchedulerBasicP$McuSleep$sleep(void);
# 50 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
enum SchedulerBasicP$__nesc_unnamed4275 {

  SchedulerBasicP$NUM_TASKS = 7U, 
  SchedulerBasicP$NO_TASK = 255
};

volatile uint8_t SchedulerBasicP$m_head;
volatile uint8_t SchedulerBasicP$m_tail;
volatile uint8_t SchedulerBasicP$m_next[SchedulerBasicP$NUM_TASKS];








static __inline uint8_t SchedulerBasicP$popTask(void);
#line 86
static inline bool SchedulerBasicP$isWaiting(uint8_t id);




static inline bool SchedulerBasicP$pushTask(uint8_t id);
#line 113
static inline  void SchedulerBasicP$Scheduler$init(void);









static  bool SchedulerBasicP$Scheduler$runNextTask(void);
#line 138
static inline  void SchedulerBasicP$Scheduler$taskLoop(void);
#line 159
static   error_t SchedulerBasicP$TaskBasic$postTask(uint8_t id);




static   void SchedulerBasicP$TaskBasic$default$runTask(uint8_t id);
# 54 "/opt/tinyos-2.x/tos/interfaces/McuPowerOverride.nc"
static   mcu_power_t McuSleepC$McuPowerOverride$lowestState(void);
# 51 "/opt/tinyos-2.x/tos/chips/msp430/McuSleepC.nc"
bool McuSleepC$dirty = TRUE;
mcu_power_t McuSleepC$powerState = MSP430_POWER_ACTIVE;




const uint16_t McuSleepC$msp430PowerBits[MSP430_POWER_LPM4 + 1] = { 
0, 
0x0010, 
0x0040 + 0x0010, 
0x0080 + 0x0010, 
0x0080 + 0x0040 + 0x0010, 
0x0080 + 0x0040 + 0x0020 + 0x0010 };


static inline mcu_power_t McuSleepC$getPowerState(void);
#line 104
static inline void McuSleepC$computePowerState(void);




static inline   void McuSleepC$McuSleep$sleep(void);
#line 124
static inline    mcu_power_t McuSleepC$McuPowerOverride$default$lowestState(void);
# 55 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static  error_t LightSensorM$Read$read(void);
# 53 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static  void LightSensorM$T1$startPeriodic(uint32_t arg_0x1390c10);
# 56 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
static   void LightSensorM$Leds$led0Toggle(void);
#line 123
static   void LightSensorM$Leds$set(uint8_t arg_0x1395630);
# 81 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static  bool LightSensorM$T2$isRunning(void);
#line 62
static  void LightSensorM$T2$startOneShot(uint32_t arg_0x13a81e8);
# 14 "LightSensorM.nc"
static inline  void LightSensorM$Boot$booted(void);







static inline  void LightSensorM$T1$fired(void);



static inline  void LightSensorM$T2$fired(void);



static  void LightSensorM$Read$readDone(error_t result, uint16_t data);
# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static   void LedsP$Led0$toggle(void);



static   void LedsP$Led0$makeOutput(void);
#line 29
static   void LedsP$Led0$set(void);
static   void LedsP$Led0$clr(void);




static   void LedsP$Led1$makeOutput(void);
#line 29
static   void LedsP$Led1$set(void);
static   void LedsP$Led1$clr(void);




static   void LedsP$Led2$makeOutput(void);
#line 29
static   void LedsP$Led2$set(void);
static   void LedsP$Led2$clr(void);
# 45 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static inline  error_t LedsP$Init$init(void);
#line 63
static inline   void LedsP$Leds$led0On(void);




static inline   void LedsP$Leds$led0Off(void);




static inline   void LedsP$Leds$led0Toggle(void);




static inline   void LedsP$Leds$led1On(void);




static inline   void LedsP$Leds$led1Off(void);









static inline   void LedsP$Leds$led2On(void);




static inline   void LedsP$Leds$led2Off(void);
#line 125
static inline   void LedsP$Leds$set(uint8_t val);
# 45 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$set(void);
static inline   void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$clr(void);
static inline   void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$toggle(void);




static inline   void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$makeOutput(void);
#line 45
static inline   void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$set(void);
static inline   void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$clr(void);





static inline   void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$makeOutput(void);
#line 45
static inline   void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$set(void);
static inline   void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$clr(void);





static inline   void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$makeOutput(void);
#line 50
static inline   void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$makeInput(void);



static inline   void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$selectModuleFunc(void);

static inline   void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$selectIOFunc(void);
#line 50
static inline   void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$makeInput(void);



static inline   void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$selectModuleFunc(void);

static inline   void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$selectIOFunc(void);
#line 50
static inline   void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$makeInput(void);



static inline   void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$selectModuleFunc(void);

static inline   void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$selectIOFunc(void);
#line 50
static inline   void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$makeInput(void);



static inline   void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$selectModuleFunc(void);

static inline   void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$selectIOFunc(void);
#line 50
static inline   void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$makeInput(void);



static inline   void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$selectModuleFunc(void);

static inline   void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$selectIOFunc(void);
#line 50
static inline   void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$makeInput(void);



static inline   void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$selectModuleFunc(void);

static inline   void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$selectIOFunc(void);
#line 50
static inline   void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$makeInput(void);



static inline   void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$selectModuleFunc(void);

static inline   void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$selectIOFunc(void);
#line 50
static inline   void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$makeInput(void);



static inline   void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$selectModuleFunc(void);

static inline   void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$selectIOFunc(void);
# 44 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$toggle(void);
#line 71
static   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$makeOutput(void);
#line 34
static   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$set(void);




static   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$clr(void);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$set(void);
static inline   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$clr(void);
static inline   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$toggle(void);



static inline   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$makeOutput(void);
# 71 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static   void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$makeOutput(void);
#line 34
static   void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$set(void);




static   void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$clr(void);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$set(void);
static inline   void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$clr(void);




static inline   void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$makeOutput(void);
# 71 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static   void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$makeOutput(void);
#line 34
static   void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$set(void);




static   void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$clr(void);
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$set(void);
static inline   void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$clr(void);




static inline   void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$makeOutput(void);
# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
static   AdcP$ConfigReadStream$adc_config_t AdcP$ConfigReadStream$getConfiguration(
# 52 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15d7330);
# 189 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static   error_t AdcP$SingleChannelReadStream$getData(
# 53 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15d7ce8);
# 138 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static   error_t AdcP$SingleChannelReadStream$configureMultiple(
# 53 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15d7ce8, 
# 138 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t *arg_0x15d2840, uint16_t arg_0x15d29e8[], uint16_t arg_0x15d2b78, uint16_t arg_0x15d2d00);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t AdcP$ResourceReadStream$release(
# 54 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15d6848);
# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static  void AdcP$Read$readDone(
# 38 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15b2010, 
# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
error_t arg_0x13b47f0, AdcP$Read$val_t arg_0x13b4970);
# 65 "/opt/tinyos-2.x/tos/interfaces/ReadNow.nc"
static   void AdcP$ReadNow$readDone(
# 39 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15b0010, 
# 65 "/opt/tinyos-2.x/tos/interfaces/ReadNow.nc"
error_t arg_0x15c87c0, AdcP$ReadNow$val_t arg_0x15c8940);
# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
static   AdcP$Config$adc_config_t AdcP$Config$getConfiguration(
# 49 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15dba28);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static   error_t AdcP$finishStreamRequest$postTask(void);
# 189 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static   error_t AdcP$SingleChannel$getData(
# 50 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15ea740);
# 84 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static   error_t AdcP$SingleChannel$configureSingle(
# 50 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15ea740, 
# 84 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t *arg_0x15d35b8);
# 89 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
static  void AdcP$ReadStream$bufferDone(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15c61a0, 
# 89 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
error_t arg_0x1595718, 
AdcP$ReadStream$val_t *arg_0x15958c8, uint16_t arg_0x1595a50);
#line 102
static  void AdcP$ReadStream$readDone(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15c61a0, 
# 102 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
error_t arg_0x15940b0, uint32_t arg_0x1594240);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t AdcP$ResourceRead$release(
# 45 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15c0660);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t AdcP$ResourceRead$request(
# 45 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15c0660);
# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   bool AdcP$ResourceRead$isOwner(
# 45 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x15c0660);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static   error_t AdcP$signalBufferDone$postTask(void);
#line 56
static   error_t AdcP$readDone$postTask(void);
# 83 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
enum AdcP$__nesc_unnamed4276 {
#line 83
  AdcP$finishStreamRequest = 0U
};
#line 83
typedef int AdcP$__nesc_sillytask_finishStreamRequest[AdcP$finishStreamRequest];
enum AdcP$__nesc_unnamed4277 {
#line 84
  AdcP$signalBufferDone = 1U
};
#line 84
typedef int AdcP$__nesc_sillytask_signalBufferDone[AdcP$signalBufferDone];
#line 161
enum AdcP$__nesc_unnamed4278 {
#line 161
  AdcP$readDone = 2U
};
#line 161
typedef int AdcP$__nesc_sillytask_readDone[AdcP$readDone];
#line 60
enum AdcP$__nesc_unnamed4279 {
  AdcP$STATE_READ, 
  AdcP$STATE_READNOW, 
  AdcP$STATE_READNOW_INVALID_CONFIG, 
  AdcP$STATE_READSTREAM
};

struct AdcP$stream_entry_t {
  uint16_t count;
  struct AdcP$stream_entry_t *next;
};


 uint8_t AdcP$state;
 uint8_t AdcP$owner;
 uint16_t AdcP$value;
 uint16_t *AdcP$resultBuf;


 struct AdcP$stream_entry_t *AdcP$streamBuf[1U];
 uint32_t AdcP$usPeriod[1U];
msp430adc12_channel_config_t AdcP$streamConfig;





static inline error_t AdcP$configure(uint8_t client);









static inline  error_t AdcP$Read$read(uint8_t client);






static  void AdcP$ResourceRead$granted(uint8_t client);
#line 161
static inline void  AdcP$readDone$runTask(void);





static   error_t AdcP$SingleChannel$singleDataReady(uint8_t client, uint16_t data);
#line 186
static inline   uint16_t *AdcP$SingleChannel$multipleDataReady(uint8_t client, 
uint16_t *buf, uint16_t length);
#line 222
static inline void  AdcP$finishStreamRequest$runTask(void);
#line 241
static  void AdcP$ResourceReadStream$granted(uint8_t streamClient);
#line 278
static   uint16_t *AdcP$SingleChannelReadStream$multipleDataReady(uint8_t streamClient, 
uint16_t *buf, uint16_t length);
#line 312
static inline void  AdcP$signalBufferDone$runTask(void);





static inline   error_t AdcP$SingleChannelReadStream$singleDataReady(uint8_t streamClient, uint16_t data);





static inline    error_t AdcP$ResourceRead$default$request(uint8_t client);

static inline    error_t AdcP$ResourceRead$default$release(uint8_t client);
static inline    bool AdcP$ResourceRead$default$isOwner(uint8_t client);
static inline   void AdcP$Read$default$readDone(uint8_t client, error_t result, uint16_t val);





static inline    void AdcP$ReadNow$default$readDone(uint8_t client, error_t result, uint16_t val);






static inline    error_t AdcP$ResourceReadStream$default$release(uint8_t streamClient);

static inline   void AdcP$ReadStream$default$bufferDone(uint8_t streamClient, error_t result, 
uint16_t *buf, uint16_t count);
static inline   void AdcP$ReadStream$default$readDone(uint8_t streamClient, error_t result, uint32_t actualPeriod);

static inline    error_t AdcP$SingleChannel$default$getData(uint8_t client);





const msp430adc12_channel_config_t AdcP$defaultConfig = { INPUT_CHANNEL_NONE, 0, 0, 0, 0, 0, 0, 0 };
static inline    const msp430adc12_channel_config_t *
AdcP$Config$default$getConfiguration(uint8_t client);




static inline    const msp430adc12_channel_config_t *
AdcP$ConfigReadStream$default$getConfiguration(uint8_t client);




static inline    error_t AdcP$SingleChannelReadStream$default$configureMultiple(uint8_t client, 
const msp430adc12_channel_config_t *config, uint16_t buffer[], 
uint16_t numSamples, uint16_t jiffies);




static inline    error_t AdcP$SingleChannelReadStream$default$getData(uint8_t client);




static inline    error_t AdcP$SingleChannel$default$configureSingle(uint8_t client, 
const msp430adc12_channel_config_t *config);
# 110 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc"
static   void Msp430Adc12ImplP$MultiChannel$dataReady(
# 42 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x1664010, 
# 110 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc"
uint16_t *arg_0x1658e80, uint16_t arg_0x1656030);
# 63 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
static   adc12ctl0_t Msp430Adc12ImplP$HplAdc12$getCtl0(void);
#line 82
static   adc12memctl_t Msp430Adc12ImplP$HplAdc12$getMCtl(uint8_t arg_0x16747a0);
#line 106
static   void Msp430Adc12ImplP$HplAdc12$resetIFGs(void);
#line 75
static   void Msp430Adc12ImplP$HplAdc12$setMCtl(uint8_t arg_0x1674068, adc12memctl_t arg_0x16741f8);
#line 128
static   void Msp430Adc12ImplP$HplAdc12$startConversion(void);
#line 51
static   void Msp430Adc12ImplP$HplAdc12$setCtl0(adc12ctl0_t arg_0x1677e98);
#line 89
static   uint16_t Msp430Adc12ImplP$HplAdc12$getMem(uint8_t arg_0x1674d40);





static   void Msp430Adc12ImplP$HplAdc12$setIEFlags(uint16_t arg_0x1673350);
#line 123
static   void Msp430Adc12ImplP$HplAdc12$stopConversion(void);









static   void Msp430Adc12ImplP$HplAdc12$enableConversion(void);
#line 57
static   void Msp430Adc12ImplP$HplAdc12$setCtl1(adc12ctl1_t arg_0x1676400);
# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static   void Msp430Adc12ImplP$Port64$makeInput(void);
#line 85
static   void Msp430Adc12ImplP$Port64$selectIOFunc(void);
#line 78
static   void Msp430Adc12ImplP$Port64$selectModuleFunc(void);
# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void Msp430Adc12ImplP$CompareA1$setEvent(uint16_t arg_0x128c8e0);
# 35 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static   void Msp430Adc12ImplP$ControlA0$setControl(msp430_compare_control_t arg_0x12848c0);
# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static   void Msp430Adc12ImplP$Port62$makeInput(void);
#line 85
static   void Msp430Adc12ImplP$Port62$selectIOFunc(void);
#line 78
static   void Msp430Adc12ImplP$Port62$selectModuleFunc(void);
# 49 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
static   void Msp430Adc12ImplP$Overflow$memOverflow(
# 43 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x1664798);
# 54 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
static   void Msp430Adc12ImplP$Overflow$conversionTimeOverflow(
# 43 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x1664798);
# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static   void Msp430Adc12ImplP$Port67$makeInput(void);
#line 85
static   void Msp430Adc12ImplP$Port67$selectIOFunc(void);
#line 78
static   void Msp430Adc12ImplP$Port67$selectModuleFunc(void);
#line 64
static   void Msp430Adc12ImplP$Port60$makeInput(void);
#line 85
static   void Msp430Adc12ImplP$Port60$selectIOFunc(void);
#line 78
static   void Msp430Adc12ImplP$Port60$selectModuleFunc(void);
#line 64
static   void Msp430Adc12ImplP$Port65$makeInput(void);
#line 85
static   void Msp430Adc12ImplP$Port65$selectIOFunc(void);
#line 78
static   void Msp430Adc12ImplP$Port65$selectModuleFunc(void);
# 41 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   void Msp430Adc12ImplP$TimerA$clear(void);


static   void Msp430Adc12ImplP$TimerA$setClockSource(uint16_t arg_0x1279178);
#line 43
static   void Msp430Adc12ImplP$TimerA$disableEvents(void);
#line 39
static   void Msp430Adc12ImplP$TimerA$setMode(int arg_0x127b088);





static   void Msp430Adc12ImplP$TimerA$setInputDivider(uint16_t arg_0x1279620);
# 88 "/opt/tinyos-2.x/tos/interfaces/ArbiterInfo.nc"
static   uint8_t Msp430Adc12ImplP$ADCArbiterInfo$userId(void);
# 35 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static   void Msp430Adc12ImplP$ControlA1$setControl(msp430_compare_control_t arg_0x12848c0);
# 227 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static   uint16_t *Msp430Adc12ImplP$SingleChannel$multipleDataReady(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x1666508, 
# 227 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t arg_0x15eb9d0[], uint16_t arg_0x15ebb60);
#line 206
static   error_t Msp430Adc12ImplP$SingleChannel$singleDataReady(
# 41 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x1666508, 
# 206 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t arg_0x15eb230);
# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static   void Msp430Adc12ImplP$Port63$makeInput(void);
#line 85
static   void Msp430Adc12ImplP$Port63$selectIOFunc(void);
#line 78
static   void Msp430Adc12ImplP$Port63$selectModuleFunc(void);
# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void Msp430Adc12ImplP$CompareA0$setEvent(uint16_t arg_0x128c8e0);
# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static   void Msp430Adc12ImplP$Port61$makeInput(void);
#line 85
static   void Msp430Adc12ImplP$Port61$selectIOFunc(void);
#line 78
static   void Msp430Adc12ImplP$Port61$selectModuleFunc(void);
#line 64
static   void Msp430Adc12ImplP$Port66$makeInput(void);
#line 85
static   void Msp430Adc12ImplP$Port66$selectIOFunc(void);
#line 78
static   void Msp430Adc12ImplP$Port66$selectModuleFunc(void);
# 66 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
enum Msp430Adc12ImplP$__nesc_unnamed4280 {
  Msp430Adc12ImplP$SINGLE_DATA = 1, 
  Msp430Adc12ImplP$SINGLE_DATA_REPEAT = 2, 
  Msp430Adc12ImplP$MULTIPLE_DATA = 4, 
  Msp430Adc12ImplP$MULTIPLE_DATA_REPEAT = 8, 
  Msp430Adc12ImplP$MULTI_CHANNEL = 16, 
  Msp430Adc12ImplP$CONVERSION_MODE_MASK = 0x1F, 

  Msp430Adc12ImplP$ADC_BUSY = 32, 
  Msp430Adc12ImplP$USE_TIMERA = 64, 
  Msp430Adc12ImplP$ADC_OVERFLOW = 128
};

uint8_t Msp430Adc12ImplP$state;

uint16_t *Msp430Adc12ImplP$resultBuffer;
uint16_t Msp430Adc12ImplP$resultBufferLength;
uint16_t Msp430Adc12ImplP$resultBufferIndex;
uint8_t Msp430Adc12ImplP$numChannels;
uint8_t Msp430Adc12ImplP$clientID;

static inline  error_t Msp430Adc12ImplP$Init$init(void);





static inline void Msp430Adc12ImplP$prepareTimerA(uint16_t interval, uint16_t csSAMPCON, uint16_t cdSAMPCON);
#line 109
static inline void Msp430Adc12ImplP$startTimerA(void);
#line 128
static inline void Msp430Adc12ImplP$configureAdcPin(uint8_t inch);
#line 145
static void Msp430Adc12ImplP$resetAdcPin(uint8_t inch);
#line 162
static inline   error_t Msp430Adc12ImplP$SingleChannel$configureSingle(uint8_t id, 
const msp430adc12_channel_config_t *config);
#line 253
static   error_t Msp430Adc12ImplP$SingleChannel$configureMultiple(uint8_t id, 
const msp430adc12_channel_config_t *config, 
uint16_t *buf, uint16_t length, uint16_t jiffies);
#line 366
static   error_t Msp430Adc12ImplP$SingleChannel$getData(uint8_t id);
#line 471
static void Msp430Adc12ImplP$stopConversion(void);
#line 510
static inline   void Msp430Adc12ImplP$TimerA$overflow(void);
static inline   void Msp430Adc12ImplP$CompareA0$fired(void);
static inline   void Msp430Adc12ImplP$CompareA1$fired(void);

static inline   void Msp430Adc12ImplP$HplAdc12$conversionDone(uint16_t iv);
#line 597
static inline    error_t Msp430Adc12ImplP$SingleChannel$default$singleDataReady(uint8_t id, uint16_t data);




static inline    uint16_t *Msp430Adc12ImplP$SingleChannel$default$multipleDataReady(uint8_t id, 
uint16_t *buf, uint16_t length);




static inline    void Msp430Adc12ImplP$MultiChannel$default$dataReady(uint8_t id, uint16_t *buffer, uint16_t numSamples);

static inline    void Msp430Adc12ImplP$Overflow$default$memOverflow(uint8_t id);
static inline    void Msp430Adc12ImplP$Overflow$default$conversionTimeOverflow(uint8_t id);
# 112 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
static   void HplAdc12P$HplAdc12$conversionDone(uint16_t arg_0x1672010);
# 51 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
 static volatile uint16_t HplAdc12P$ADC12CTL0 __asm ("0x01A0");
 static volatile uint16_t HplAdc12P$ADC12CTL1 __asm ("0x01A2");
 static volatile uint16_t HplAdc12P$ADC12IFG __asm ("0x01A4");
 static volatile uint16_t HplAdc12P$ADC12IE __asm ("0x01A6");
 static volatile uint16_t HplAdc12P$ADC12IV __asm ("0x01A8");

static inline   void HplAdc12P$HplAdc12$setCtl0(adc12ctl0_t control0);



static inline   void HplAdc12P$HplAdc12$setCtl1(adc12ctl1_t control1);



static inline   adc12ctl0_t HplAdc12P$HplAdc12$getCtl0(void);







static inline   void HplAdc12P$HplAdc12$setMCtl(uint8_t i, adc12memctl_t memControl);





static   adc12memctl_t HplAdc12P$HplAdc12$getMCtl(uint8_t i);







static inline   uint16_t HplAdc12P$HplAdc12$getMem(uint8_t i);



static inline   void HplAdc12P$HplAdc12$setIEFlags(uint16_t mask);


static inline   void HplAdc12P$HplAdc12$resetIFGs(void);
#line 106
static inline   void HplAdc12P$HplAdc12$startConversion(void);




static inline   void HplAdc12P$HplAdc12$stopConversion(void);




static inline   void HplAdc12P$HplAdc12$enableConversion(void);



static inline   bool HplAdc12P$HplAdc12$isBusy(void);

void sig_ADC_VECTOR(void)  __attribute((wakeup)) __attribute((interrupt(14))) ;
# 39 "/opt/tinyos-2.x/tos/system/RoundRobinResourceQueueC.nc"
enum /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$__nesc_unnamed4281 {
  RoundRobinResourceQueueC$0$NO_ENTRY = 0xFF, 
  RoundRobinResourceQueueC$0$SIZE = 2U ? (2U - 1) / 8 + 1 : 0
};

uint8_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$resQ[/*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$SIZE];
uint8_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$last = 0;

static inline void /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$clearEntry(uint8_t id);



static inline  error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$Init$init(void);




static inline   bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$isEmpty(void);








static   bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$isEnqueued(resource_client_id_t id);



static inline   resource_client_id_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$dequeue(void);
#line 87
static inline   error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$enqueue(resource_client_id_t id);
# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
static   void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceRequested$requested(
# 52 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x1735168);
# 55 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static   void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$unconfigure(
# 56 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x1735c18);
# 49 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static   void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$configure(
# 56 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x1735c18);
# 69 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
static   error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Queue$enqueue(resource_client_id_t arg_0x1727358);
#line 43
static   bool /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Queue$isEmpty(void);
#line 60
static   resource_client_id_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Queue$dequeue(void);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$granted(
# 51 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x1737830);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static   error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask$postTask(void);
# 69 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
enum /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$__nesc_unnamed4282 {
#line 69
  SimpleArbiterP$0$grantedTask = 3U
};
#line 69
typedef int /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$__nesc_sillytask_grantedTask[/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask];
#line 62
enum /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$__nesc_unnamed4283 {
#line 62
  SimpleArbiterP$0$RES_IDLE = 0, SimpleArbiterP$0$RES_GRANTING = 1, SimpleArbiterP$0$RES_BUSY = 2
};
#line 63
enum /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$__nesc_unnamed4284 {
#line 63
  SimpleArbiterP$0$NO_RES = 0xFF
};
uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$state = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$RES_IDLE;
 uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$resId = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$NO_RES;
 uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$reqResId;



static   error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$request(uint8_t id);
#line 97
static   error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$release(uint8_t id);
#line 136
static inline   uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ArbiterInfo$userId(void);






static   uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$isOwner(uint8_t id);






static inline  void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask$runTask(void);









static inline   void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$default$granted(uint8_t id);

static inline    void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceRequested$default$requested(uint8_t id);



static inline    void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$configure(uint8_t id);

static inline    void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$unconfigure(uint8_t id);
# 63 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
static   adc12ctl0_t Msp430RefVoltGeneratorP$HplAdc12$getCtl0(void);
#line 118
static   bool Msp430RefVoltGeneratorP$HplAdc12$isBusy(void);
#line 51
static   void Msp430RefVoltGeneratorP$HplAdc12$setCtl0(adc12ctl0_t arg_0x1677e98);
# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static  void Msp430RefVoltGeneratorP$SwitchOffTimer$startOneShot(uint32_t arg_0x13a81e8);




static  void Msp430RefVoltGeneratorP$SwitchOffTimer$stop(void);
# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static  void Msp430RefVoltGeneratorP$RefVolt_2_5V$startDone(error_t arg_0x177c638);
#line 117
static  void Msp430RefVoltGeneratorP$RefVolt_2_5V$stopDone(error_t arg_0x177b1e0);
#line 92
static  void Msp430RefVoltGeneratorP$RefVolt_1_5V$startDone(error_t arg_0x177c638);
#line 117
static  void Msp430RefVoltGeneratorP$RefVolt_1_5V$stopDone(error_t arg_0x177b1e0);
# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static  void Msp430RefVoltGeneratorP$SwitchOnTimer$startOneShot(uint32_t arg_0x13a81e8);




static  void Msp430RefVoltGeneratorP$SwitchOnTimer$stop(void);
# 47 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
enum Msp430RefVoltGeneratorP$__nesc_unnamed4285 {

  Msp430RefVoltGeneratorP$GENERATOR_OFF, 
  Msp430RefVoltGeneratorP$REFERENCE_1_5V_PENDING, 
  Msp430RefVoltGeneratorP$REFERENCE_2_5V_PENDING, 
  Msp430RefVoltGeneratorP$REFERENCE_1_5V_STABLE, 
  Msp430RefVoltGeneratorP$REFERENCE_2_5V_STABLE
};

uint8_t Msp430RefVoltGeneratorP$state;

static error_t Msp430RefVoltGeneratorP$switchOn(uint8_t level);
#line 78
static error_t Msp430RefVoltGeneratorP$switchOff(void);
#line 94
static inline  error_t Msp430RefVoltGeneratorP$RefVolt_1_5V$start(void);
#line 127
static inline  error_t Msp430RefVoltGeneratorP$RefVolt_1_5V$stop(void);
#line 157
static inline  error_t Msp430RefVoltGeneratorP$RefVolt_2_5V$start(void);
#line 220
static inline  void Msp430RefVoltGeneratorP$SwitchOnTimer$fired(void);
#line 244
static inline  void Msp430RefVoltGeneratorP$SwitchOffTimer$fired(void);
#line 274
static inline   void Msp430RefVoltGeneratorP$HplAdc12$conversionDone(uint16_t iv);
# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
static   Msp430RefVoltArbiterImplP$Config$adc_config_t Msp430RefVoltArbiterImplP$Config$getConfiguration(
# 43 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x1795e58);
# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static  error_t Msp430RefVoltArbiterImplP$RefVolt_2_5V$start(void);
# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t Msp430RefVoltArbiterImplP$AdcResource$release(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x1797888);
# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   error_t Msp430RefVoltArbiterImplP$AdcResource$request(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x1797888);
# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static   bool Msp430RefVoltArbiterImplP$AdcResource$isOwner(
# 40 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x1797888);
# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
static  void Msp430RefVoltArbiterImplP$ClientResource$granted(
# 38 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x1798ee0);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static   error_t Msp430RefVoltArbiterImplP$switchOff$postTask(void);
# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
static  error_t Msp430RefVoltArbiterImplP$RefVolt_1_5V$start(void);
#line 109
static  error_t Msp430RefVoltArbiterImplP$RefVolt_1_5V$stop(void);
# 51 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
enum Msp430RefVoltArbiterImplP$__nesc_unnamed4286 {
#line 51
  Msp430RefVoltArbiterImplP$switchOff = 4U
};
#line 51
typedef int Msp430RefVoltArbiterImplP$__nesc_sillytask_switchOff[Msp430RefVoltArbiterImplP$switchOff];
#line 46
enum Msp430RefVoltArbiterImplP$__nesc_unnamed4287 {
  Msp430RefVoltArbiterImplP$NO_OWNER = 0xFF
};
 uint8_t Msp430RefVoltArbiterImplP$syncOwner = Msp430RefVoltArbiterImplP$NO_OWNER;



static inline   error_t Msp430RefVoltArbiterImplP$ClientResource$request(uint8_t client);
#line 70
static  void Msp430RefVoltArbiterImplP$AdcResource$granted(uint8_t client);
#line 98
static inline  void Msp430RefVoltArbiterImplP$RefVolt_1_5V$startDone(error_t error);








static  void Msp430RefVoltArbiterImplP$RefVolt_2_5V$startDone(error_t error);








static   error_t Msp430RefVoltArbiterImplP$ClientResource$release(uint8_t client);
#line 136
static inline  void Msp430RefVoltArbiterImplP$switchOff$runTask(void);










static inline  void Msp430RefVoltArbiterImplP$RefVolt_1_5V$stopDone(error_t error);



static inline  void Msp430RefVoltArbiterImplP$RefVolt_2_5V$stopDone(error_t error);



static inline   uint8_t Msp430RefVoltArbiterImplP$ClientResource$isOwner(uint8_t client);




static inline   void Msp430RefVoltArbiterImplP$ClientResource$default$granted(uint8_t client);
static inline    error_t Msp430RefVoltArbiterImplP$AdcResource$default$request(uint8_t client);







static inline    bool Msp430RefVoltArbiterImplP$AdcResource$default$isOwner(uint8_t client);
static inline    error_t Msp430RefVoltArbiterImplP$AdcResource$default$release(uint8_t client);
const msp430adc12_channel_config_t Msp430RefVoltArbiterImplP$defaultConfig = { INPUT_CHANNEL_NONE, 0, 0, 0, 0, 0, 0, 0 };
static inline    const msp430adc12_channel_config_t *
Msp430RefVoltArbiterImplP$Config$default$getConfiguration(uint8_t client);
# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$setEvent(uint16_t arg_0x128c8e0);

static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$setEventFromNow(uint16_t arg_0x128b248);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$get(void);
# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$fired(void);
# 39 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$enableEvents(void);
#line 36
static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$setControlAsCompare(void);



static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$disableEvents(void);
#line 33
static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$clearPendingInterrupt(void);
# 42 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline  error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Init$init(void);
#line 54
static inline   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$stop(void);




static inline   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$fired(void);










static inline   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$startAt(uint16_t t0, uint16_t dt);
#line 103
static inline   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$overflow(void);
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static   uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$get(void);
static   bool /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$isOverflowPending(void);
# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static   void /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$overflow(void);
# 38 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline   uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$get(void);




static inline   bool /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$isOverflowPending(void);









static inline   void /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$overflow(void);
# 53 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static   /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$size_type /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$get(void);






static   bool /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$isOverflowPending(void);










static   void /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$overflow(void);
# 56 "/opt/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
/*CounterMilli32C.Transform*/TransformCounterC$0$upper_count_type /*CounterMilli32C.Transform*/TransformCounterC$0$m_upper;

enum /*CounterMilli32C.Transform*/TransformCounterC$0$__nesc_unnamed4288 {

  TransformCounterC$0$LOW_SHIFT_RIGHT = 5, 
  TransformCounterC$0$HIGH_SHIFT_LEFT = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC$0$from_size_type ) - /*CounterMilli32C.Transform*/TransformCounterC$0$LOW_SHIFT_RIGHT, 
  TransformCounterC$0$NUM_UPPER_BITS = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type ) - 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC$0$from_size_type ) + 5, 



  TransformCounterC$0$OVERFLOW_MASK = /*CounterMilli32C.Transform*/TransformCounterC$0$NUM_UPPER_BITS ? ((/*CounterMilli32C.Transform*/TransformCounterC$0$upper_count_type )2 << (/*CounterMilli32C.Transform*/TransformCounterC$0$NUM_UPPER_BITS - 1)) - 1 : 0
};

static   /*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$get(void);
#line 122
static inline   void /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$overflow(void);
# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$fired(void);
#line 92
static   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$size_type arg_0x17d8aa8, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$size_type arg_0x17d8c30);
#line 62
static   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$stop(void);
# 53 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
static   /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$get(void);
# 66 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0;
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_dt;

enum /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$__nesc_unnamed4289 {

  TransformAlarmC$0$MAX_DELAY_LOG2 = 8 * sizeof(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$from_size_type ) - 1 - 5, 
  TransformAlarmC$0$MAX_DELAY = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type )1 << /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$MAX_DELAY_LOG2
};

static inline   /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getNow(void);




static inline   /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getAlarm(void);










static inline   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$stop(void);




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$set_alarm(void);
#line 136
static   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type dt);
#line 151
static inline   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$fired(void);
#line 166
static inline   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$overflow(void);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static   error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$postTask(void);
# 98 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
static   /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$getNow(void);
#line 92
static   void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type arg_0x17d8aa8, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type arg_0x17d8c30);
#line 105
static   /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$getAlarm(void);
#line 62
static   void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$stop(void);
# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static  void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$fired(void);
# 63 "/opt/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
enum /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$__nesc_unnamed4290 {
#line 63
  AlarmToTimerC$0$fired = 5U
};
#line 63
typedef int /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$__nesc_sillytask_fired[/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired];
#line 44
uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_dt;
bool /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_oneshot;

static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$start(uint32_t t0, uint32_t dt, bool oneshot);
#line 60
static inline  void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$stop(void);


static inline  void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$runTask(void);






static inline   void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$fired(void);
#line 82
static inline  void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$startOneShotAt(uint32_t t0, uint32_t dt);


static inline  uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$getNow(void);
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static   error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$postTask(void);
# 125 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
static  uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow(void);
#line 118
static  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$startOneShotAt(uint32_t arg_0x13a6df8, uint32_t arg_0x13a5010);
#line 67
static  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$stop(void);




static  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$fired(
# 37 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x190d3c0);
#line 60
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$__nesc_unnamed4291 {
#line 60
  VirtualizeTimerC$0$updateFromTimer = 6U
};
#line 60
typedef int /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$__nesc_sillytask_updateFromTimer[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer];
#line 42
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$__nesc_unnamed4292 {

  VirtualizeTimerC$0$NUM_TIMERS = 4U, 
  VirtualizeTimerC$0$END_OF_LIST = 255
};








#line 48
typedef struct /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$__nesc_unnamed4293 {

  uint32_t t0;
  uint32_t dt;
  bool isoneshot : 1;
  bool isrunning : 1;
  bool _reserved : 6;
} /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer_t;

/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_timers[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$NUM_TIMERS];




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$fireTimers(uint32_t now);
#line 88
static inline  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$runTask(void);
#line 127
static inline  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$fired(void);




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot);
#line 146
static inline  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startPeriodic(uint8_t num, uint32_t dt);




static  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShot(uint8_t num, uint32_t dt);




static inline  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$stop(uint8_t num);




static inline  bool /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$isRunning(uint8_t num);
#line 196
static inline   void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$default$fired(uint8_t num);
# 49 "/opt/tinyos-2.x/tos/sensorboards/hy2420/LightP.nc"
msp430adc12_channel_config_t LightP$config = { 
.inch = INPUT_CHANNEL_A4, 
.sref = REFERENCE_AVcc_AVss, 
.ref2_5v = REFVOLT_LEVEL_1_5, 
.adc12ssel = SHT_SOURCE_ACLK, 
.adc12div = SHT_CLOCK_DIV_1, 
.sht = SAMPLE_HOLD_4_CYCLES, 
.sampcon_ssel = SAMPCON_SOURCE_SMCLK, 
.sampcon_id = SAMPCON_CLOCK_DIV_1 };





static inline   const msp430adc12_channel_config_t *LightP$AdcConfigure$getConfiguration(void);
# 196 "/opt/tinyos-2.x/tos/chips/msp430/msp430hardware.h"
static inline void __nesc_enable_interrupt(void )
{
   __asm volatile ("eint");}

# 185 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Timer$overflow(void)
{
}

#line 185
static inline   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Timer$overflow(void)
{
}

#line 185
static inline   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Timer$overflow(void)
{
}

# 510 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline   void Msp430Adc12ImplP$TimerA$overflow(void)
#line 510
{
}

# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$overflow(void){
#line 37
  Msp430Adc12ImplP$TimerA$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Timer$overflow();
#line 37
}
#line 37
# 126 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Overflow$fired(void)
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$overflow();
}





static inline    void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$default$fired(uint8_t n)
{
}

# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$fired(uint8_t arg_0x12aa9a8){
#line 28
  switch (arg_0x12aa9a8) {
#line 28
    case 0:
#line 28
      /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Event$fired();
#line 28
      break;
#line 28
    case 1:
#line 28
      /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Event$fired();
#line 28
      break;
#line 28
    case 2:
#line 28
      /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Event$fired();
#line 28
      break;
#line 28
    case 5:
#line 28
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Overflow$fired();
#line 28
      break;
#line 28
    default:
#line 28
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$default$fired(arg_0x12aa9a8);
#line 28
      break;
#line 28
    }
#line 28
}
#line 28
# 115 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX0$fired(void)
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$fired(0);
}

# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static   void Msp430TimerCommonP$VectorTimerA0$fired(void){
#line 28
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX0$fired();
#line 28
}
#line 28
# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$int2CC(uint16_t x)
#line 47
{
#line 47
  union  {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline   /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Control$getControl(void)
{
  return /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$int2CC(* (volatile uint16_t *)354U);
}

#line 177
static inline    void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$captured(uint16_t arg_0x1294d38){
#line 75
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$default$captured(arg_0x1294d38);
#line 75
}
#line 75
# 139 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$getEvent(void)
{
  return * (volatile uint16_t *)370U;
}

# 511 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline   void Msp430Adc12ImplP$CompareA0$fired(void)
#line 511
{
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$fired(void){
#line 34
  Msp430Adc12ImplP$CompareA0$fired();
#line 34
}
#line 34
# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$int2CC(uint16_t x)
#line 47
{
#line 47
  union  {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline   /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Control$getControl(void)
{
  return /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$int2CC(* (volatile uint16_t *)356U);
}

#line 177
static inline    void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$captured(uint16_t arg_0x1294d38){
#line 75
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$default$captured(arg_0x1294d38);
#line 75
}
#line 75
# 139 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$getEvent(void)
{
  return * (volatile uint16_t *)372U;
}

# 512 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline   void Msp430Adc12ImplP$CompareA1$fired(void)
#line 512
{
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$fired(void){
#line 34
  Msp430Adc12ImplP$CompareA1$fired();
#line 34
}
#line 34
# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$int2CC(uint16_t x)
#line 47
{
#line 47
  union  {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline   /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Control$getControl(void)
{
  return /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$int2CC(* (volatile uint16_t *)358U);
}

#line 177
static inline    void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static   void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$captured(uint16_t arg_0x1294d38){
#line 75
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$default$captured(arg_0x1294d38);
#line 75
}
#line 75
# 139 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$getEvent(void)
{
  return * (volatile uint16_t *)374U;
}

#line 181
static inline    void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$default$fired(void)
{
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static   void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$fired(void){
#line 34
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$default$fired();
#line 34
}
#line 34
# 120 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX1$fired(void)
{
  uint8_t n = * (volatile uint16_t *)302U;

#line 123
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$fired(n >> 1);
}

# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static   void Msp430TimerCommonP$VectorTimerA1$fired(void){
#line 28
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX1$fired();
#line 28
}
#line 28
# 115 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline   void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX0$fired(void)
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$fired(0);
}

# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static   void Msp430TimerCommonP$VectorTimerB0$fired(void){
#line 28
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX0$fired();
#line 28
}
#line 28
# 185 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Timer$overflow(void)
{
}

#line 185
static inline   void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Timer$overflow(void)
{
}

#line 185
static inline   void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Timer$overflow(void)
{
}

#line 185
static inline   void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Timer$overflow(void)
{
}

#line 185
static inline   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Timer$overflow(void)
{
}

#line 185
static inline   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Timer$overflow(void)
{
}

#line 185
static inline   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$overflow(void)
{
}

# 103 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$overflow(void)
{
}

# 166 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$overflow(void)
{
}

# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static   void /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$overflow(void){
#line 71
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$overflow();
#line 71
}
#line 71
# 122 "/opt/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
static inline   void /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$overflow(void)
{
  /* atomic removed: atomic calls only */
  {
    /*CounterMilli32C.Transform*/TransformCounterC$0$m_upper++;
    if ((/*CounterMilli32C.Transform*/TransformCounterC$0$m_upper & /*CounterMilli32C.Transform*/TransformCounterC$0$OVERFLOW_MASK) == 0) {
      /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$overflow();
      }
  }
}

# 71 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static   void /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$overflow(void){
#line 71
  /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$overflow();
#line 71
}
#line 71
# 53 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline   void /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$overflow(void)
{
  /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$overflow();
}

# 37 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static   void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$overflow(void){
#line 37
  /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$overflow();
#line 37
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Timer$overflow();
#line 37
}
#line 37
# 126 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline   void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Overflow$fired(void)
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$overflow();
}

# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static   error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$postTask(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 70 "/opt/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline   void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$fired(void)
{
#line 71
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$postTask();
}

# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$fired(void){
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$fired();
#line 67
}
#line 67
# 151 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$fired(void)
{
  /* atomic removed: atomic calls only */
  {
    if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_dt == 0) 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$fired();
      }
    else 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$set_alarm();
      }
  }
}

# 67 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$fired(void){
#line 67
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$fired();
#line 67
}
#line 67
# 124 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$disableEvents(void)
{
  * (volatile uint16_t *)386U &= ~0x0010;
}

# 40 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$disableEvents(void){
#line 40
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$disableEvents();
#line 40
}
#line 40
# 59 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$fired(void)
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$fired();
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$fired(void){
#line 34
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$getEvent(void)
{
  return * (volatile uint16_t *)402U;
}

#line 177
static inline    void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$captured(uint16_t arg_0x1294d38){
#line 75
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$default$captured(arg_0x1294d38);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$int2CC(uint16_t x)
#line 47
{
#line 47
  union  {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline   /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$getControl(void)
{
  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$int2CC(* (volatile uint16_t *)386U);
}

#line 169
static inline   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Event$fired(void)
{
  if (/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$captured(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$fired();
    }
}

# 86 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static inline bool SchedulerBasicP$isWaiting(uint8_t id)
{
  return SchedulerBasicP$m_next[id] != SchedulerBasicP$NO_TASK || SchedulerBasicP$m_tail == id;
}

static inline bool SchedulerBasicP$pushTask(uint8_t id)
{
  if (!SchedulerBasicP$isWaiting(id)) 
    {
      if (SchedulerBasicP$m_head == SchedulerBasicP$NO_TASK) 
        {
          SchedulerBasicP$m_head = id;
          SchedulerBasicP$m_tail = id;
        }
      else 
        {
          SchedulerBasicP$m_next[SchedulerBasicP$m_tail] = id;
          SchedulerBasicP$m_tail = id;
        }
      return TRUE;
    }
  else 
    {
      return FALSE;
    }
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static   uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$get(void){
#line 34
  unsigned int result;
#line 34

#line 34
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$get();
#line 34

#line 34
  return result;
#line 34
}
#line 34
# 38 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline   uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$get(void)
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$get();
}

# 53 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static   /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$size_type /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$get(void){
#line 53
  unsigned int result;
#line 53

#line 53
  result = /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$get();
#line 53

#line 53
  return result;
#line 53
}
#line 53
# 70 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline   bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$isOverflowPending(void)
{
  return * (volatile uint16_t *)384U & 1U;
}

# 35 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static   bool /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$isOverflowPending(void){
#line 35
  unsigned char result;
#line 35

#line 35
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$isOverflowPending();
#line 35

#line 35
  return result;
#line 35
}
#line 35
# 43 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline   bool /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$isOverflowPending(void)
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$isOverflowPending();
}

# 60 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static   bool /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$isOverflowPending(void){
#line 60
  unsigned char result;
#line 60

#line 60
  result = /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$isOverflowPending();
#line 60

#line 60
  return result;
#line 60
}
#line 60
# 119 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$enableEvents(void)
{
  * (volatile uint16_t *)386U |= 0x0010;
}

# 39 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$enableEvents(void){
#line 39
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$enableEvents();
#line 39
}
#line 39
# 84 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$clearPendingInterrupt(void)
{
  * (volatile uint16_t *)386U &= ~0x0001;
}

# 33 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$clearPendingInterrupt(void){
#line 33
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$clearPendingInterrupt();
#line 33
}
#line 33
# 144 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEvent(uint16_t x)
{
  * (volatile uint16_t *)402U = x;
}

# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$setEvent(uint16_t arg_0x128c8e0){
#line 30
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEvent(arg_0x128c8e0);
#line 30
}
#line 30
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static   uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$get(void){
#line 34
  unsigned int result;
#line 34

#line 34
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$get();
#line 34

#line 34
  return result;
#line 34
}
#line 34
# 154 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEventFromNow(uint16_t x)
{
  * (volatile uint16_t *)402U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$get() + x;
}

# 32 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$setEventFromNow(uint16_t arg_0x128b248){
#line 32
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEventFromNow(arg_0x128b248);
#line 32
}
#line 32
# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static   uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$get(void){
#line 34
  unsigned int result;
#line 34

#line 34
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$get();
#line 34

#line 34
  return result;
#line 34
}
#line 34
# 70 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$get();
    uint16_t elapsed = now - t0;

#line 76
    if (elapsed >= dt) 
      {
        /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 83
        if (remaining <= 2) {
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$setEventFromNow(2);
          }
        else {
#line 86
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$setEvent(now + remaining);
          }
      }
#line 88
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$clearPendingInterrupt();
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$enableEvents();
  }
}

# 92 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$size_type arg_0x17d8aa8, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$size_type arg_0x17d8c30){
#line 92
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$startAt(arg_0x17d8aa8, arg_0x17d8c30);
#line 92
}
#line 92
# 181 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline    void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$default$fired(void)
{
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$fired(void){
#line 34
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$default$fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$getEvent(void)
{
  return * (volatile uint16_t *)404U;
}

#line 177
static inline    void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$captured(uint16_t arg_0x1294d38){
#line 75
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$default$captured(arg_0x1294d38);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$int2CC(uint16_t x)
#line 47
{
#line 47
  union  {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline   /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$getControl(void)
{
  return /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$int2CC(* (volatile uint16_t *)388U);
}

#line 169
static inline   void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Event$fired(void)
{
  if (/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$captured(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$fired();
    }
}




static inline    void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$default$fired(void)
{
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$fired(void){
#line 34
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$default$fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$getEvent(void)
{
  return * (volatile uint16_t *)406U;
}

#line 177
static inline    void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$captured(uint16_t arg_0x1294d38){
#line 75
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$default$captured(arg_0x1294d38);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$int2CC(uint16_t x)
#line 47
{
#line 47
  union  {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline   /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$getControl(void)
{
  return /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$int2CC(* (volatile uint16_t *)390U);
}

#line 169
static inline   void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Event$fired(void)
{
  if (/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$captured(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$fired();
    }
}




static inline    void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$default$fired(void)
{
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static   void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$fired(void){
#line 34
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$default$fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$getEvent(void)
{
  return * (volatile uint16_t *)408U;
}

#line 177
static inline    void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static   void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$captured(uint16_t arg_0x1294d38){
#line 75
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$default$captured(arg_0x1294d38);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$int2CC(uint16_t x)
#line 47
{
#line 47
  union  {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline   /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Control$getControl(void)
{
  return /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$int2CC(* (volatile uint16_t *)392U);
}

#line 169
static inline   void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Event$fired(void)
{
  if (/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$captured(/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$fired();
    }
}




static inline    void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$default$fired(void)
{
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static   void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$fired(void){
#line 34
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$default$fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$getEvent(void)
{
  return * (volatile uint16_t *)410U;
}

#line 177
static inline    void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static   void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$captured(uint16_t arg_0x1294d38){
#line 75
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$default$captured(arg_0x1294d38);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$int2CC(uint16_t x)
#line 47
{
#line 47
  union  {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline   /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Control$getControl(void)
{
  return /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$int2CC(* (volatile uint16_t *)394U);
}

#line 169
static inline   void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Event$fired(void)
{
  if (/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$captured(/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$fired();
    }
}




static inline    void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$default$fired(void)
{
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static   void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$fired(void){
#line 34
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$default$fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$getEvent(void)
{
  return * (volatile uint16_t *)412U;
}

#line 177
static inline    void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static   void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$captured(uint16_t arg_0x1294d38){
#line 75
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$default$captured(arg_0x1294d38);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$int2CC(uint16_t x)
#line 47
{
#line 47
  union  {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline   /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Control$getControl(void)
{
  return /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$int2CC(* (volatile uint16_t *)396U);
}

#line 169
static inline   void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Event$fired(void)
{
  if (/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$captured(/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$fired();
    }
}




static inline    void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$default$fired(void)
{
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static   void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$fired(void){
#line 34
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$default$fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$getEvent(void)
{
  return * (volatile uint16_t *)414U;
}

#line 177
static inline    void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static   void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$captured(uint16_t arg_0x1294d38){
#line 75
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$default$captured(arg_0x1294d38);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$int2CC(uint16_t x)
#line 47
{
#line 47
  union __nesc_unnamed4294 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline   /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Control$getControl(void)
{
  return /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$int2CC(* (volatile uint16_t *)398U);
}

#line 169
static inline   void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Event$fired(void)
{
  if (/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$captured(/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$fired();
    }
}

# 120 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline   void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX1$fired(void)
{
  uint8_t n = * (volatile uint16_t *)286U;

#line 123
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$fired(n >> 1);
}

# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static   void Msp430TimerCommonP$VectorTimerB1$fired(void){
#line 28
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX1$fired();
#line 28
}
#line 28
# 113 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static inline  void SchedulerBasicP$Scheduler$init(void)
{
  /* atomic removed: atomic calls only */
  {
    memset((void *)SchedulerBasicP$m_next, SchedulerBasicP$NO_TASK, sizeof SchedulerBasicP$m_next);
    SchedulerBasicP$m_head = SchedulerBasicP$NO_TASK;
    SchedulerBasicP$m_tail = SchedulerBasicP$NO_TASK;
  }
}

# 46 "/opt/tinyos-2.x/tos/interfaces/Scheduler.nc"
inline static  void RealMainP$Scheduler$init(void){
#line 46
  SchedulerBasicP$Scheduler$init();
#line 46
}
#line 46
# 45 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$set(void)
#line 45
{
  /* atomic removed: atomic calls only */
#line 45
  * (volatile uint8_t *)49U |= 0x01 << 6;
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$set(void){
#line 34
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$set();
#line 34
}
#line 34
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$set(void)
#line 37
{
#line 37
  /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$set();
}

# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void LedsP$Led2$set(void){
#line 29
  /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$set();
#line 29
}
#line 29
# 45 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$set(void)
#line 45
{
  /* atomic removed: atomic calls only */
#line 45
  * (volatile uint8_t *)49U |= 0x01 << 5;
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$set(void){
#line 34
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$set();
#line 34
}
#line 34
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$set(void)
#line 37
{
#line 37
  /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$set();
}

# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void LedsP$Led1$set(void){
#line 29
  /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$set();
#line 29
}
#line 29
# 45 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$set(void)
#line 45
{
  /* atomic removed: atomic calls only */
#line 45
  * (volatile uint8_t *)49U |= 0x01 << 4;
}

# 34 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$set(void){
#line 34
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$set();
#line 34
}
#line 34
# 37 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$set(void)
#line 37
{
#line 37
  /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$set();
}

# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void LedsP$Led0$set(void){
#line 29
  /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$set();
#line 29
}
#line 29
# 52 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$makeOutput(void)
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t *)50U |= 0x01 << 6;
}

# 71 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$makeOutput(void){
#line 71
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$makeOutput(void)
#line 43
{
#line 43
  /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$makeOutput();
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void LedsP$Led2$makeOutput(void){
#line 35
  /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$makeOutput();
#line 35
}
#line 35
# 52 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$makeOutput(void)
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t *)50U |= 0x01 << 5;
}

# 71 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$makeOutput(void){
#line 71
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$makeOutput(void)
#line 43
{
#line 43
  /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$makeOutput();
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void LedsP$Led1$makeOutput(void){
#line 35
  /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$makeOutput();
#line 35
}
#line 35
# 52 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$makeOutput(void)
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t *)50U |= 0x01 << 4;
}

# 71 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$makeOutput(void){
#line 71
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$makeOutput(void)
#line 43
{
#line 43
  /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$makeOutput();
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void LedsP$Led0$makeOutput(void){
#line 35
  /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$makeOutput();
#line 35
}
#line 35
# 45 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static inline  error_t LedsP$Init$init(void)
#line 45
{
  /* atomic removed: atomic calls only */
#line 46
  {
    ;
    LedsP$Led0$makeOutput();
    LedsP$Led1$makeOutput();
    LedsP$Led2$makeOutput();
    LedsP$Led0$set();
    LedsP$Led1$set();
    LedsP$Led2$set();
  }
  return SUCCESS;
}

# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
inline static  error_t PlatformP$LedsInit$init(void){
#line 51
  unsigned char result;
#line 51

#line 51
  result = LedsP$Init$init();
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 79 "/opt/tinyos-2.x/tos/platforms/hmote2420/hardware.h"
static inline void TOSH_CLR_FLASH_HOLD_PIN(void)
#line 79
{
#line 79
   static volatile uint8_t r __asm ("0x001D");

#line 79
  r &= ~(1 << 7);
}

#line 32
static inline void TOSH_MAKE_UCLK0_INPUT(void)
#line 32
{
#line 32
   static volatile uint8_t r __asm ("0x001A");

#line 32
  r &= ~(1 << 3);
}

#line 31
static inline void TOSH_MAKE_SIMO0_INPUT(void)
#line 31
{
#line 31
   static volatile uint8_t r __asm ("0x001A");

#line 31
  r &= ~(1 << 1);
}

#line 31
static inline void TOSH_SET_SIMO0_PIN(void)
#line 31
{
#line 31
   static volatile uint8_t r __asm ("0x0019");

#line 31
  r |= 1 << 1;
}

#line 78
static inline void TOSH_SET_FLASH_CS_PIN(void)
#line 78
{
#line 78
   static volatile uint8_t r __asm ("0x001D");

#line 78
  r |= 1 << 4;
}

#line 32
static inline void TOSH_CLR_UCLK0_PIN(void)
#line 32
{
#line 32
   static volatile uint8_t r __asm ("0x0019");

#line 32
  r &= ~(1 << 3);
}

#line 78
static inline void TOSH_CLR_FLASH_CS_PIN(void)
#line 78
{
#line 78
   static volatile uint8_t r __asm ("0x001D");

#line 78
  r &= ~(1 << 4);
}

# 11 "/opt/tinyos-2.x/tos/platforms/hmote2420/MotePlatformC.nc"
static __inline void MotePlatformC$TOSH_wait(void)
#line 11
{
   __asm volatile ("nop"); __asm volatile ("nop");}

# 79 "/opt/tinyos-2.x/tos/platforms/hmote2420/hardware.h"
static inline void TOSH_SET_FLASH_HOLD_PIN(void)
#line 79
{
#line 79
   static volatile uint8_t r __asm ("0x001D");

#line 79
  r |= 1 << 7;
}

#line 78
static inline void TOSH_MAKE_FLASH_CS_OUTPUT(void)
#line 78
{
#line 78
   static volatile uint8_t r __asm ("0x001E");

#line 78
  r |= 1 << 4;
}

#line 79
static inline void TOSH_MAKE_FLASH_HOLD_OUTPUT(void)
#line 79
{
#line 79
   static volatile uint8_t r __asm ("0x001E");

#line 79
  r |= 1 << 7;
}

#line 32
static inline void TOSH_MAKE_UCLK0_OUTPUT(void)
#line 32
{
#line 32
   static volatile uint8_t r __asm ("0x001A");

#line 32
  r |= 1 << 3;
}

#line 31
static inline void TOSH_MAKE_SIMO0_OUTPUT(void)
#line 31
{
#line 31
   static volatile uint8_t r __asm ("0x001A");

#line 31
  r |= 1 << 1;
}

# 27 "/opt/tinyos-2.x/tos/platforms/hmote2420/MotePlatformC.nc"
static inline void MotePlatformC$TOSH_FLASH_M25P_DP(void)
#line 27
{

  TOSH_MAKE_SIMO0_OUTPUT();
  TOSH_MAKE_UCLK0_OUTPUT();
  TOSH_MAKE_FLASH_HOLD_OUTPUT();
  TOSH_MAKE_FLASH_CS_OUTPUT();
  TOSH_SET_FLASH_HOLD_PIN();
  TOSH_SET_FLASH_CS_PIN();

  MotePlatformC$TOSH_wait();


  TOSH_CLR_FLASH_CS_PIN();
  TOSH_CLR_UCLK0_PIN();

  MotePlatformC$TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC$TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC$TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC$TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC$TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC$TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC$TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC$TOSH_FLASH_M25P_DP_bit(TRUE);

  TOSH_SET_FLASH_CS_PIN();

  TOSH_SET_SIMO0_PIN();
  TOSH_MAKE_SIMO0_INPUT();
  TOSH_MAKE_UCLK0_INPUT();
  TOSH_CLR_FLASH_HOLD_PIN();
}

#line 6
static __inline void MotePlatformC$uwait(uint16_t u)
#line 6
{
  uint16_t t0 = TA0R;

#line 8
  while (TA0R - t0 <= u) ;
}

#line 59
static inline  error_t MotePlatformC$Init$init(void)
#line 59
{
  /* atomic removed: atomic calls only */

  {
    P1SEL = 0;
    P2SEL = 0;
    P3SEL = 0;
    P4SEL = 0;
    P5SEL = 0;
    P6SEL = 0;

    P1DIR = 0xe0;
    P1OUT = 0x00;

    P2DIR = 0x7b;
    P2OUT = 0x30;

    P3DIR = 0xf1;
    P3OUT = 0x00;

    P4DIR = 0xfd;
    P4OUT = 0xdd;

    P5DIR = 0xff;
    P5OUT = 0xff;

    P6DIR = 0xff;
    P6OUT = 0x00;

    P1IE = 0;
    P2IE = 0;






    MotePlatformC$uwait(1024 * 10);

    MotePlatformC$TOSH_FLASH_M25P_DP();
  }

  return SUCCESS;
}

# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
inline static  error_t PlatformP$MoteInit$init(void){
#line 51
  unsigned char result;
#line 51

#line 51
  result = MotePlatformC$Init$init();
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 130 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP$startTimerB(void)
{

  Msp430ClockP$TBCTL = 0x0020 | (Msp430ClockP$TBCTL & ~(0x0020 | 0x0010));
}

#line 118
static inline void Msp430ClockP$startTimerA(void)
{

  Msp430ClockP$TA0CTL = 0x0020 | (Msp430ClockP$TA0CTL & ~(0x0020 | 0x0010));
}

#line 87
static inline  void Msp430ClockP$Msp430ClockInit$defaultInitTimerB(void)
{
  TBR = 0;









  Msp430ClockP$TBCTL = 0x0100 | 0x0002;
}











static inline   void Msp430ClockP$Msp430ClockInit$default$initTimerB(void)
{
  Msp430ClockP$Msp430ClockInit$defaultInitTimerB();
}

# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static  void Msp430ClockP$Msp430ClockInit$initTimerB(void){
#line 30
  Msp430ClockP$Msp430ClockInit$default$initTimerB();
#line 30
}
#line 30
# 72 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline  void Msp430ClockP$Msp430ClockInit$defaultInitTimerA(void)
{
  TA0R = 0;









  Msp430ClockP$TA0CTL = 0x0200 | 0x0002;
}

#line 107
static inline   void Msp430ClockP$Msp430ClockInit$default$initTimerA(void)
{
  Msp430ClockP$Msp430ClockInit$defaultInitTimerA();
}

# 29 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static  void Msp430ClockP$Msp430ClockInit$initTimerA(void){
#line 29
  Msp430ClockP$Msp430ClockInit$default$initTimerA();
#line 29
}
#line 29
# 51 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline  void Msp430ClockP$Msp430ClockInit$defaultInitClocks(void)
{





  BCSCTL1 = 0x80 | (BCSCTL1 & ((0x04 | 0x02) | 0x01));







  BCSCTL2 = 0x04;


  Msp430ClockP$IE1 &= ~(1 << 1);
}

#line 102
static inline   void Msp430ClockP$Msp430ClockInit$default$initClocks(void)
{
  Msp430ClockP$Msp430ClockInit$defaultInitClocks();
}

# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static  void Msp430ClockP$Msp430ClockInit$initClocks(void){
#line 28
  Msp430ClockP$Msp430ClockInit$default$initClocks();
#line 28
}
#line 28
# 148 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline uint16_t Msp430ClockP$test_calib_busywait_delta(int calib)
{
  int8_t aclk_count = 2;
  uint16_t dco_prev = 0;
  uint16_t dco_curr = 0;

  Msp430ClockP$set_dco_calib(calib);

  while (aclk_count-- > 0) 
    {
      TBCCR0 = TBR + Msp430ClockP$ACLK_CALIB_PERIOD;
      TBCCTL0 &= ~0x0001;
      while ((TBCCTL0 & 0x0001) == 0) ;
      dco_prev = dco_curr;
      dco_curr = TA0R;
    }

  return dco_curr - dco_prev;
}




static inline void Msp430ClockP$busyCalibrateDco(void)
{

  int calib;
  int step;



  Msp430ClockP$TA0CTL = 0x0200 | 0x0020;
  Msp430ClockP$TBCTL = 0x0100 | 0x0020;
  BCSCTL1 = 0x80 | 0x04;
  BCSCTL2 = 0;
  TBCCTL0 = 0x4000;






  for (calib = 0, step = 0x800; step != 0; step >>= 1) 
    {

      if (Msp430ClockP$test_calib_busywait_delta(calib | step) <= Msp430ClockP$TARGET_DCO_DELTA) {
        calib |= step;
        }
    }

  if ((calib & 0x0e0) == 0x0e0) {
    calib &= ~0x01f;
    }
  Msp430ClockP$set_dco_calib(calib);
}

static inline  error_t Msp430ClockP$Init$init(void)
{

  Msp430ClockP$TA0CTL = 0x0004;
  Msp430ClockP$TA0IV = 0;
  Msp430ClockP$TBCTL = 0x0004;
  Msp430ClockP$TBIV = 0;
  /* atomic removed: atomic calls only */

  {
    Msp430ClockP$busyCalibrateDco();
    Msp430ClockP$Msp430ClockInit$initClocks();
    Msp430ClockP$Msp430ClockInit$initTimerA();
    Msp430ClockP$Msp430ClockInit$initTimerB();
    Msp430ClockP$startTimerA();
    Msp430ClockP$startTimerB();
  }

  return SUCCESS;
}

# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
inline static  error_t PlatformP$Msp430ClockInit$init(void){
#line 51
  unsigned char result;
#line 51

#line 51
  result = Msp430ClockP$Init$init();
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 10 "/opt/tinyos-2.x/tos/platforms/hmote2420/PlatformP.nc"
static inline  error_t PlatformP$Init$init(void)
#line 10
{
  PlatformP$Msp430ClockInit$init();
  PlatformP$MoteInit$init();
  PlatformP$LedsInit$init();
  return SUCCESS;
}

# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
inline static  error_t RealMainP$PlatformInit$init(void){
#line 51
  unsigned char result;
#line 51

#line 51
  result = PlatformP$Init$init();
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 31 "/opt/tinyos-2.x/tos/platforms/hmote2420/hardware.h"
static inline void TOSH_CLR_SIMO0_PIN(void)
#line 31
{
#line 31
   static volatile uint8_t r __asm ("0x0019");

#line 31
  r &= ~(1 << 1);
}

#line 32
static inline void TOSH_SET_UCLK0_PIN(void)
#line 32
{
#line 32
   static volatile uint8_t r __asm ("0x0019");

#line 32
  r |= 1 << 3;
}

# 54 "/opt/tinyos-2.x/tos/interfaces/Scheduler.nc"
inline static  bool RealMainP$Scheduler$runNextTask(void){
#line 54
  unsigned char result;
#line 54

#line 54
  result = SchedulerBasicP$Scheduler$runNextTask();
#line 54

#line 54
  return result;
#line 54
}
#line 54
# 92 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static   void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type arg_0x17d8aa8, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type arg_0x17d8c30){
#line 92
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$startAt(arg_0x17d8aa8, arg_0x17d8c30);
#line 92
}
#line 92
# 47 "/opt/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$start(uint32_t t0, uint32_t dt, bool oneshot)
{
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_dt = dt;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_oneshot = oneshot;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$startAt(t0, dt);
}

#line 82
static inline  void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$startOneShotAt(uint32_t t0, uint32_t dt)
{
#line 83
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$start(t0, dt, TRUE);
}

# 118 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$startOneShotAt(uint32_t arg_0x13a6df8, uint32_t arg_0x13a5010){
#line 118
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$startOneShotAt(arg_0x13a6df8, arg_0x13a5010);
#line 118
}
#line 118
# 54 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$stop(void)
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$disableEvents();
}

# 62 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$stop(void){
#line 62
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$stop();
#line 62
}
#line 62
# 91 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$stop(void)
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$stop();
}

# 62 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static   void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$stop(void){
#line 62
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$stop();
#line 62
}
#line 62
# 60 "/opt/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline  void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$stop(void)
{
#line 61
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$stop();
}

# 67 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$stop(void){
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$stop();
#line 67
}
#line 67
# 53 "/opt/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static   /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$get(void){
#line 53
  unsigned long result;
#line 53

#line 53
  result = /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$get();
#line 53

#line 53
  return result;
#line 53
}
#line 53
# 75 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline   /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getNow(void)
{
  return /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$get();
}

# 98 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static   /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$getNow(void){
#line 98
  unsigned long result;
#line 98

#line 98
  result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getNow();
#line 98

#line 98
  return result;
#line 98
}
#line 98
# 85 "/opt/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline  uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$getNow(void)
{
#line 86
  return /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$getNow();
}

# 125 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static  uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow(void){
#line 125
  unsigned long result;
#line 125

#line 125
  result = /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$getNow();
#line 125

#line 125
  return result;
#line 125
}
#line 125
# 88 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$runTask(void)
{




  uint32_t now = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow();
  int32_t min_remaining = (1UL << 31) - 1;
  bool min_remaining_isset = FALSE;
  uint8_t num;

  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$stop();

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;
          int32_t remaining = timer->dt - elapsed;

          if (remaining < min_remaining) 
            {
              min_remaining = remaining;
              min_remaining_isset = TRUE;
            }
        }
    }

  if (min_remaining_isset) 
    {
      if (min_remaining <= 0) {
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$fireTimers(now);
        }
      else {
#line 123
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$startOneShotAt(now, min_remaining);
        }
    }
}

# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static  void Msp430RefVoltGeneratorP$RefVolt_2_5V$startDone(error_t arg_0x177c638){
#line 92
  Msp430RefVoltArbiterImplP$RefVolt_2_5V$startDone(arg_0x177c638);
#line 92
}
#line 92
# 160 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline   void Msp430RefVoltArbiterImplP$ClientResource$default$granted(uint8_t client)
#line 160
{
}

# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static  void Msp430RefVoltArbiterImplP$ClientResource$granted(uint8_t arg_0x1798ee0){
#line 92
  switch (arg_0x1798ee0) {
#line 92
    case /*LightSensorC.Sensor.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID:
#line 92
      AdcP$ResourceRead$granted(/*LightSensorC.Sensor.AdcReadClientC*/AdcReadClientC$0$CLIENT);
#line 92
      break;
#line 92
    case /*LightSensorC.Sensor.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$1$ID:
#line 92
      AdcP$ResourceReadStream$granted(/*LightSensorC.Sensor.AdcReadStreamClientC*/AdcReadStreamClientC$0$RSCLIENT);
#line 92
      break;
#line 92
    default:
#line 92
      Msp430RefVoltArbiterImplP$ClientResource$default$granted(arg_0x1798ee0);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 98 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline  void Msp430RefVoltArbiterImplP$RefVolt_1_5V$startDone(error_t error)
{
  if (Msp430RefVoltArbiterImplP$syncOwner != Msp430RefVoltArbiterImplP$NO_OWNER) {


      Msp430RefVoltArbiterImplP$ClientResource$granted(Msp430RefVoltArbiterImplP$syncOwner);
    }
}

# 92 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static  void Msp430RefVoltGeneratorP$RefVolt_1_5V$startDone(error_t arg_0x177c638){
#line 92
  Msp430RefVoltArbiterImplP$RefVolt_1_5V$startDone(arg_0x177c638);
#line 92
}
#line 92
# 220 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline  void Msp430RefVoltGeneratorP$SwitchOnTimer$fired(void)
{
  switch (Msp430RefVoltGeneratorP$state) 
    {
      case Msp430RefVoltGeneratorP$REFERENCE_1_5V_PENDING: 
        Msp430RefVoltGeneratorP$state = Msp430RefVoltGeneratorP$REFERENCE_1_5V_STABLE;
      Msp430RefVoltGeneratorP$RefVolt_1_5V$startDone(SUCCESS);
      break;
      case Msp430RefVoltGeneratorP$REFERENCE_2_5V_PENDING: 
        Msp430RefVoltGeneratorP$state = Msp430RefVoltGeneratorP$REFERENCE_2_5V_STABLE;
      Msp430RefVoltGeneratorP$RefVolt_2_5V$startDone(SUCCESS);
      break;
      case Msp430RefVoltGeneratorP$REFERENCE_1_5V_STABLE: 

        case Msp430RefVoltGeneratorP$GENERATOR_OFF: 

          case Msp430RefVoltGeneratorP$REFERENCE_2_5V_STABLE: 

            default: 

              return;
    }
}

# 62 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static  void Msp430RefVoltGeneratorP$SwitchOffTimer$startOneShot(uint32_t arg_0x13a81e8){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShot(1U, arg_0x13a81e8);
#line 62
}
#line 62
# 151 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline  void Msp430RefVoltArbiterImplP$RefVolt_2_5V$stopDone(error_t error)
{
}

# 117 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static  void Msp430RefVoltGeneratorP$RefVolt_2_5V$stopDone(error_t arg_0x177b1e0){
#line 117
  Msp430RefVoltArbiterImplP$RefVolt_2_5V$stopDone(arg_0x177b1e0);
#line 117
}
#line 117
# 147 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline  void Msp430RefVoltArbiterImplP$RefVolt_1_5V$stopDone(error_t error)
{
}

# 117 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static  void Msp430RefVoltGeneratorP$RefVolt_1_5V$stopDone(error_t arg_0x177b1e0){
#line 117
  Msp430RefVoltArbiterImplP$RefVolt_1_5V$stopDone(arg_0x177b1e0);
#line 117
}
#line 117
# 244 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline  void Msp430RefVoltGeneratorP$SwitchOffTimer$fired(void)
{
  switch (Msp430RefVoltGeneratorP$state) 
    {
      case Msp430RefVoltGeneratorP$REFERENCE_1_5V_STABLE: 
        if (Msp430RefVoltGeneratorP$switchOff() == SUCCESS) {
            Msp430RefVoltGeneratorP$state = Msp430RefVoltGeneratorP$GENERATOR_OFF;
            Msp430RefVoltGeneratorP$RefVolt_1_5V$stopDone(SUCCESS);
          }
        else {
#line 253
          Msp430RefVoltGeneratorP$SwitchOffTimer$startOneShot(20);
          }
#line 254
      break;
      case Msp430RefVoltGeneratorP$REFERENCE_2_5V_STABLE: 
        if (Msp430RefVoltGeneratorP$switchOff() == SUCCESS) {
            Msp430RefVoltGeneratorP$state = Msp430RefVoltGeneratorP$GENERATOR_OFF;
            Msp430RefVoltGeneratorP$RefVolt_2_5V$stopDone(SUCCESS);
          }
        else {
#line 260
          Msp430RefVoltGeneratorP$SwitchOffTimer$startOneShot(20);
          }
#line 261
      break;
      case Msp430RefVoltGeneratorP$GENERATOR_OFF: 

        case Msp430RefVoltGeneratorP$REFERENCE_1_5V_PENDING: 

          case Msp430RefVoltGeneratorP$REFERENCE_2_5V_PENDING: 

            default: 

              return;
    }
}

# 161 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline    error_t Msp430RefVoltArbiterImplP$AdcResource$default$request(uint8_t client)
{
  return FAIL;
}

# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   error_t Msp430RefVoltArbiterImplP$AdcResource$request(uint8_t arg_0x1797888){
#line 78
  unsigned char result;
#line 78

#line 78
  switch (arg_0x1797888) {
#line 78
    case /*LightSensorC.Sensor.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID:
#line 78
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$request(/*LightSensorC.Sensor.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID);
#line 78
      break;
#line 78
    case /*LightSensorC.Sensor.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$1$ID:
#line 78
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$request(/*LightSensorC.Sensor.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$1$ID);
#line 78
      break;
#line 78
    default:
#line 78
      result = Msp430RefVoltArbiterImplP$AdcResource$default$request(arg_0x1797888);
#line 78
      break;
#line 78
    }
#line 78

#line 78
  return result;
#line 78
}
#line 78
# 53 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline   error_t Msp430RefVoltArbiterImplP$ClientResource$request(uint8_t client)
{
  return Msp430RefVoltArbiterImplP$AdcResource$request(client);
}

# 324 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline    error_t AdcP$ResourceRead$default$request(uint8_t client)
#line 324
{
#line 324
  return FAIL;
}

# 78 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   error_t AdcP$ResourceRead$request(uint8_t arg_0x15c0660){
#line 78
  unsigned char result;
#line 78

#line 78
  switch (arg_0x15c0660) {
#line 78
    case /*LightSensorC.Sensor.AdcReadClientC*/AdcReadClientC$0$CLIENT:
#line 78
      result = Msp430RefVoltArbiterImplP$ClientResource$request(/*LightSensorC.Sensor.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID);
#line 78
      break;
#line 78
    default:
#line 78
      result = AdcP$ResourceRead$default$request(arg_0x15c0660);
#line 78
      break;
#line 78
    }
#line 78

#line 78
  return result;
#line 78
}
#line 78
# 169 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline    bool Msp430RefVoltArbiterImplP$AdcResource$default$isOwner(uint8_t client)
#line 169
{
#line 169
  return FALSE;
}

# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   bool Msp430RefVoltArbiterImplP$AdcResource$isOwner(uint8_t arg_0x1797888){
#line 118
  unsigned char result;
#line 118

#line 118
  switch (arg_0x1797888) {
#line 118
    case /*LightSensorC.Sensor.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID:
#line 118
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$isOwner(/*LightSensorC.Sensor.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID);
#line 118
      break;
#line 118
    case /*LightSensorC.Sensor.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$1$ID:
#line 118
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$isOwner(/*LightSensorC.Sensor.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$1$ID);
#line 118
      break;
#line 118
    default:
#line 118
      result = Msp430RefVoltArbiterImplP$AdcResource$default$isOwner(arg_0x1797888);
#line 118
      break;
#line 118
    }
#line 118

#line 118
  return result;
#line 118
}
#line 118
# 155 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline   uint8_t Msp430RefVoltArbiterImplP$ClientResource$isOwner(uint8_t client)
{
  return Msp430RefVoltArbiterImplP$AdcResource$isOwner(client);
}

# 327 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline    bool AdcP$ResourceRead$default$isOwner(uint8_t client)
#line 327
{
#line 327
  return FALSE;
}

# 118 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   bool AdcP$ResourceRead$isOwner(uint8_t arg_0x15c0660){
#line 118
  unsigned char result;
#line 118

#line 118
  switch (arg_0x15c0660) {
#line 118
    case /*LightSensorC.Sensor.AdcReadClientC*/AdcReadClientC$0$CLIENT:
#line 118
      result = Msp430RefVoltArbiterImplP$ClientResource$isOwner(/*LightSensorC.Sensor.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID);
#line 118
      break;
#line 118
    default:
#line 118
      result = AdcP$ResourceRead$default$isOwner(arg_0x15c0660);
#line 118
      break;
#line 118
    }
#line 118

#line 118
  return result;
#line 118
}
#line 118
# 97 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline  error_t AdcP$Read$read(uint8_t client)
{
  if (AdcP$ResourceRead$isOwner(client)) {
    return EBUSY;
    }
#line 101
  return AdcP$ResourceRead$request(client);
}

# 55 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
inline static  error_t LightSensorM$Read$read(void){
#line 55
  unsigned char result;
#line 55

#line 55
  result = AdcP$Read$read(/*LightSensorC.Sensor.AdcReadClientC*/AdcReadClientC$0$CLIENT);
#line 55

#line 55
  return result;
#line 55
}
#line 55
# 22 "LightSensorM.nc"
static inline  void LightSensorM$T1$fired(void)
#line 22
{
  LightSensorM$Read$read();
}

# 47 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$toggle(void)
#line 47
{
#line 47
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 47
    * (volatile uint8_t *)49U ^= 0x01 << 4;
#line 47
    __nesc_atomic_end(__nesc_atomic); }
}

# 44 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$toggle(void){
#line 44
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$toggle();
#line 44
}
#line 44
# 39 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$toggle(void)
#line 39
{
#line 39
  /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$toggle();
}

# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void LedsP$Led0$toggle(void){
#line 31
  /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$toggle();
#line 31
}
#line 31
# 73 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static inline   void LedsP$Leds$led0Toggle(void)
#line 73
{
  LedsP$Led0$toggle();
  ;
#line 75
  ;
}

# 56 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
inline static   void LightSensorM$Leds$led0Toggle(void){
#line 56
  LedsP$Leds$led0Toggle();
#line 56
}
#line 56
# 26 "LightSensorM.nc"
static inline  void LightSensorM$T2$fired(void)
#line 26
{
  LightSensorM$Leds$led0Toggle();
}

# 196 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline   void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$default$fired(uint8_t num)
{
}

# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$fired(uint8_t arg_0x190d3c0){
#line 72
  switch (arg_0x190d3c0) {
#line 72
    case 0U:
#line 72
      Msp430RefVoltGeneratorP$SwitchOnTimer$fired();
#line 72
      break;
#line 72
    case 1U:
#line 72
      Msp430RefVoltGeneratorP$SwitchOffTimer$fired();
#line 72
      break;
#line 72
    case 2U:
#line 72
      LightSensorM$T1$fired();
#line 72
      break;
#line 72
    case 3U:
#line 72
      LightSensorM$T2$fired();
#line 72
      break;
#line 72
    default:
#line 72
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$default$fired(arg_0x190d3c0);
#line 72
      break;
#line 72
    }
#line 72
}
#line 72
# 162 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static inline    void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceRequested$default$requested(uint8_t id)
#line 162
{
}

# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
inline static   void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceRequested$requested(uint8_t arg_0x1735168){
#line 43
    /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceRequested$default$requested(arg_0x1735168);
#line 43
}
#line 43
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static   error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask$postTask(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 87 "/opt/tinyos-2.x/tos/system/RoundRobinResourceQueueC.nc"
static inline   error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$enqueue(resource_client_id_t id)
#line 87
{
  /* atomic removed: atomic calls only */
#line 88
  {
    if (!/*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$isEnqueued(id)) {
        /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$resQ[id / 8] |= 1 << id % 8;
        {
          unsigned char __nesc_temp = 
#line 91
          SUCCESS;

#line 91
          return __nesc_temp;
        }
      }
#line 93
    {
      unsigned char __nesc_temp = 
#line 93
      EBUSY;

#line 93
      return __nesc_temp;
    }
  }
}

# 69 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static   error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Queue$enqueue(resource_client_id_t arg_0x1727358){
#line 69
  unsigned char result;
#line 69

#line 69
  result = /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$enqueue(arg_0x1727358);
#line 69

#line 69
  return result;
#line 69
}
#line 69
# 120 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline   bool HplAdc12P$HplAdc12$isBusy(void)
#line 120
{
#line 120
  return HplAdc12P$ADC12CTL1 & 0x0001;
}

# 118 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static   bool Msp430RefVoltGeneratorP$HplAdc12$isBusy(void){
#line 118
  unsigned char result;
#line 118

#line 118
  result = HplAdc12P$HplAdc12$isBusy();
#line 118

#line 118
  return result;
#line 118
}
#line 118
# 65 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline   adc12ctl0_t HplAdc12P$HplAdc12$getCtl0(void)
#line 65
{
  return * (adc12ctl0_t *)&HplAdc12P$ADC12CTL0;
}

# 63 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static   adc12ctl0_t Msp430RefVoltGeneratorP$HplAdc12$getCtl0(void){
#line 63
  struct __nesc_unnamed4254 result;
#line 63

#line 63
  result = HplAdc12P$HplAdc12$getCtl0();
#line 63

#line 63
  return result;
#line 63
}
#line 63
# 57 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline   void HplAdc12P$HplAdc12$setCtl0(adc12ctl0_t control0)
#line 57
{
  HplAdc12P$ADC12CTL0 = * (uint16_t *)&control0;
}

# 51 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static   void Msp430RefVoltGeneratorP$HplAdc12$setCtl0(adc12ctl0_t arg_0x1677e98){
#line 51
  HplAdc12P$HplAdc12$setCtl0(arg_0x1677e98);
#line 51
}
#line 51
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static   error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$postTask(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 63 "/opt/tinyos-2.x/tos/sensorboards/hy2420/LightP.nc"
static inline   const msp430adc12_channel_config_t *LightP$AdcConfigure$getConfiguration(void)
#line 63
{
  return &LightP$config;
}

# 360 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline    const msp430adc12_channel_config_t *
AdcP$ConfigReadStream$default$getConfiguration(uint8_t client)
{
  return &AdcP$defaultConfig;
}

# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
inline static   AdcP$ConfigReadStream$adc_config_t AdcP$ConfigReadStream$getConfiguration(uint8_t arg_0x15d7330){
#line 58
  struct __nesc_unnamed4267 const *result;
#line 58

#line 58
  switch (arg_0x15d7330) {
#line 58
    case /*LightSensorC.Sensor.AdcReadStreamClientC*/AdcReadStreamClientC$0$RSCLIENT:
#line 58
      result = LightP$AdcConfigure$getConfiguration();
#line 58
      break;
#line 58
    default:
#line 58
      result = AdcP$ConfigReadStream$default$getConfiguration(arg_0x15d7330);
#line 58
      break;
#line 58
    }
#line 58

#line 58
  return result;
#line 58
}
#line 58
# 366 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline    error_t AdcP$SingleChannelReadStream$default$configureMultiple(uint8_t client, 
const msp430adc12_channel_config_t *config, uint16_t buffer[], 
uint16_t numSamples, uint16_t jiffies)
{
  return FAIL;
}

# 138 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static   error_t AdcP$SingleChannelReadStream$configureMultiple(uint8_t arg_0x15d7ce8, const msp430adc12_channel_config_t *arg_0x15d2840, uint16_t arg_0x15d29e8[], uint16_t arg_0x15d2b78, uint16_t arg_0x15d2d00){
#line 138
  unsigned char result;
#line 138

#line 138
  switch (arg_0x15d7ce8) {
#line 138
    case /*LightSensorC.Sensor.AdcReadStreamClientC*/AdcReadStreamClientC$0$RSCLIENT:
#line 138
      result = Msp430Adc12ImplP$SingleChannel$configureMultiple(/*LightSensorC.Sensor.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$1$ID, arg_0x15d2840, arg_0x15d29e8, arg_0x15d2b78, arg_0x15d2d00);
#line 138
      break;
#line 138
    default:
#line 138
      result = AdcP$SingleChannelReadStream$default$configureMultiple(arg_0x15d7ce8, arg_0x15d2840, arg_0x15d29e8, arg_0x15d2b78, arg_0x15d2d00);
#line 138
      break;
#line 138
    }
#line 138

#line 138
  return result;
#line 138
}
#line 138
# 144 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$setEvent(uint16_t x)
{
  * (volatile uint16_t *)372U = x;
}

# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static   void Msp430Adc12ImplP$CompareA1$setEvent(uint16_t arg_0x128c8e0){
#line 30
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$setEvent(arg_0x128c8e0);
#line 30
}
#line 30
# 144 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$setEvent(uint16_t x)
{
  * (volatile uint16_t *)370U = x;
}

# 30 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static   void Msp430Adc12ImplP$CompareA0$setEvent(uint16_t arg_0x128c8e0){
#line 30
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$setEvent(arg_0x128c8e0);
#line 30
}
#line 30
# 46 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$CC2int(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t x)
#line 46
{
#line 46
  union  {
#line 46
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t f;
#line 46
    uint16_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

#line 89
static inline   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Control$setControl(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t x)
{
  * (volatile uint16_t *)354U = /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$CC2int(x);
}

# 35 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static   void Msp430Adc12ImplP$ControlA0$setControl(msp430_compare_control_t arg_0x12848c0){
#line 35
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Control$setControl(arg_0x12848c0);
#line 35
}
#line 35
# 110 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setInputDivider(uint16_t inputDivider)
{
  * (volatile uint16_t *)352U = (* (volatile uint16_t *)352U & ~(0x0040 | 0x0080)) | ((inputDivider << 6) & (0x0040 | 0x0080));
}

# 45 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static   void Msp430Adc12ImplP$TimerA$setInputDivider(uint16_t arg_0x1279620){
#line 45
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setInputDivider(arg_0x1279620);
#line 45
}
#line 45
# 105 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setClockSource(uint16_t clockSource)
{
  * (volatile uint16_t *)352U = (* (volatile uint16_t *)352U & ~(256U | 512U)) | ((clockSource << 8) & (256U | 512U));
}

# 44 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static   void Msp430Adc12ImplP$TimerA$setClockSource(uint16_t arg_0x1279178){
#line 44
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setClockSource(arg_0x1279178);
#line 44
}
#line 44
# 100 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$disableEvents(void)
{
  * (volatile uint16_t *)352U &= ~2U;
}

# 43 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static   void Msp430Adc12ImplP$TimerA$disableEvents(void){
#line 43
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$disableEvents();
#line 43
}
#line 43
# 90 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$clear(void)
{
  * (volatile uint16_t *)352U |= 4U;
}

# 41 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static   void Msp430Adc12ImplP$TimerA$clear(void){
#line 41
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$clear();
#line 41
}
#line 41
#line 39
inline static   void Msp430Adc12ImplP$TimerA$setMode(int arg_0x127b088){
#line 39
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setMode(arg_0x127b088);
#line 39
}
#line 39
# 93 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP$prepareTimerA(uint16_t interval, uint16_t csSAMPCON, uint16_t cdSAMPCON)
{
  msp430_compare_control_t ccResetSHI = { 
  .ccifg = 0, .cov = 0, .out = 0, .cci = 0, .ccie = 0, 
  .outmod = 0, .cap = 0, .clld = 0, .scs = 0, .ccis = 0, .cm = 0 };

  Msp430Adc12ImplP$TimerA$setMode(MSP430TIMER_STOP_MODE);
  Msp430Adc12ImplP$TimerA$clear();
  Msp430Adc12ImplP$TimerA$disableEvents();
  Msp430Adc12ImplP$TimerA$setClockSource(csSAMPCON);
  Msp430Adc12ImplP$TimerA$setInputDivider(cdSAMPCON);
  Msp430Adc12ImplP$ControlA0$setControl(ccResetSHI);
  Msp430Adc12ImplP$CompareA0$setEvent(interval - 1);
  Msp430Adc12ImplP$CompareA1$setEvent((interval - 1) / 2);
}

# 373 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline    error_t AdcP$SingleChannelReadStream$default$getData(uint8_t client)
{
  return FAIL;
}

# 189 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static   error_t AdcP$SingleChannelReadStream$getData(uint8_t arg_0x15d7ce8){
#line 189
  unsigned char result;
#line 189

#line 189
  switch (arg_0x15d7ce8) {
#line 189
    case /*LightSensorC.Sensor.AdcReadStreamClientC*/AdcReadStreamClientC$0$RSCLIENT:
#line 189
      result = Msp430Adc12ImplP$SingleChannel$getData(/*LightSensorC.Sensor.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$1$ID);
#line 189
      break;
#line 189
    default:
#line 189
      result = AdcP$SingleChannelReadStream$default$getData(arg_0x15d7ce8);
#line 189
      break;
#line 189
    }
#line 189

#line 189
  return result;
#line 189
}
#line 189
# 50 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$makeInput(void)
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t *)54U &= ~(0x01 << 7);
}

# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port67$makeInput(void){
#line 64
  /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$makeInput();
#line 64
}
#line 64
# 54 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$selectModuleFunc(void)
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t *)55U |= 0x01 << 7;
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port67$selectModuleFunc(void){
#line 78
  /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$selectModuleFunc();
#line 78
}
#line 78
# 50 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$makeInput(void)
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t *)54U &= ~(0x01 << 6);
}

# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port66$makeInput(void){
#line 64
  /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$makeInput();
#line 64
}
#line 64
# 54 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$selectModuleFunc(void)
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t *)55U |= 0x01 << 6;
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port66$selectModuleFunc(void){
#line 78
  /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$selectModuleFunc();
#line 78
}
#line 78
# 50 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$makeInput(void)
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t *)54U &= ~(0x01 << 5);
}

# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port65$makeInput(void){
#line 64
  /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$makeInput();
#line 64
}
#line 64
# 54 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$selectModuleFunc(void)
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t *)55U |= 0x01 << 5;
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port65$selectModuleFunc(void){
#line 78
  /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$selectModuleFunc();
#line 78
}
#line 78
# 50 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$makeInput(void)
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t *)54U &= ~(0x01 << 4);
}

# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port64$makeInput(void){
#line 64
  /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$makeInput();
#line 64
}
#line 64
# 54 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$selectModuleFunc(void)
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t *)55U |= 0x01 << 4;
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port64$selectModuleFunc(void){
#line 78
  /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$selectModuleFunc();
#line 78
}
#line 78
# 50 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$makeInput(void)
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t *)54U &= ~(0x01 << 3);
}

# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port63$makeInput(void){
#line 64
  /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$makeInput();
#line 64
}
#line 64
# 54 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$selectModuleFunc(void)
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t *)55U |= 0x01 << 3;
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port63$selectModuleFunc(void){
#line 78
  /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$selectModuleFunc();
#line 78
}
#line 78
# 50 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$makeInput(void)
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t *)54U &= ~(0x01 << 2);
}

# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port62$makeInput(void){
#line 64
  /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$makeInput();
#line 64
}
#line 64
# 54 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$selectModuleFunc(void)
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t *)55U |= 0x01 << 2;
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port62$selectModuleFunc(void){
#line 78
  /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$selectModuleFunc();
#line 78
}
#line 78
# 50 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$makeInput(void)
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t *)54U &= ~(0x01 << 1);
}

# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port61$makeInput(void){
#line 64
  /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$makeInput();
#line 64
}
#line 64
# 54 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$selectModuleFunc(void)
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t *)55U |= 0x01 << 1;
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port61$selectModuleFunc(void){
#line 78
  /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$selectModuleFunc();
#line 78
}
#line 78
# 50 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$makeInput(void)
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t *)54U &= ~(0x01 << 0);
}

# 64 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port60$makeInput(void){
#line 64
  /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$makeInput();
#line 64
}
#line 64
# 54 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$selectModuleFunc(void)
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t *)55U |= 0x01 << 0;
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port60$selectModuleFunc(void){
#line 78
  /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$selectModuleFunc();
#line 78
}
#line 78
# 128 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP$configureAdcPin(uint8_t inch)
{

  switch (inch) 
    {
      case 0: Msp430Adc12ImplP$Port60$selectModuleFunc();
#line 133
      Msp430Adc12ImplP$Port60$makeInput();
#line 133
      break;
      case 1: Msp430Adc12ImplP$Port61$selectModuleFunc();
#line 134
      Msp430Adc12ImplP$Port61$makeInput();
#line 134
      break;
      case 2: Msp430Adc12ImplP$Port62$selectModuleFunc();
#line 135
      Msp430Adc12ImplP$Port62$makeInput();
#line 135
      break;
      case 3: Msp430Adc12ImplP$Port63$selectModuleFunc();
#line 136
      Msp430Adc12ImplP$Port63$makeInput();
#line 136
      break;
      case 4: Msp430Adc12ImplP$Port64$selectModuleFunc();
#line 137
      Msp430Adc12ImplP$Port64$makeInput();
#line 137
      break;
      case 5: Msp430Adc12ImplP$Port65$selectModuleFunc();
#line 138
      Msp430Adc12ImplP$Port65$makeInput();
#line 138
      break;
      case 6: Msp430Adc12ImplP$Port66$selectModuleFunc();
#line 139
      Msp430Adc12ImplP$Port66$makeInput();
#line 139
      break;
      case 7: Msp430Adc12ImplP$Port67$selectModuleFunc();
#line 140
      Msp430Adc12ImplP$Port67$makeInput();
#line 140
      break;
    }
}

# 106 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline   void HplAdc12P$HplAdc12$startConversion(void)
#line 106
{
  HplAdc12P$ADC12CTL0 |= 0x0010;
  HplAdc12P$ADC12CTL0 |= 0x0001 + 0x0002;
}

# 128 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static   void Msp430Adc12ImplP$HplAdc12$startConversion(void){
#line 128
  HplAdc12P$HplAdc12$startConversion();
#line 128
}
#line 128
# 46 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$CC2int(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t x)
#line 46
{
#line 46
  union  {
#line 46
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t f;
#line 46
    uint16_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

#line 89
static inline   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Control$setControl(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t x)
{
  * (volatile uint16_t *)356U = /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$CC2int(x);
}

# 35 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static   void Msp430Adc12ImplP$ControlA1$setControl(msp430_compare_control_t arg_0x12848c0){
#line 35
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Control$setControl(arg_0x12848c0);
#line 35
}
#line 35
# 109 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP$startTimerA(void)
{
  msp430_compare_control_t ccSetSHI = { 
  .ccifg = 0, .cov = 0, .out = 1, .cci = 0, .ccie = 0, 
  .outmod = 0, .cap = 0, .clld = 0, .scs = 0, .ccis = 0, .cm = 0 };
  msp430_compare_control_t ccResetSHI = { 
  .ccifg = 0, .cov = 0, .out = 0, .cci = 0, .ccie = 0, 
  .outmod = 0, .cap = 0, .clld = 0, .scs = 0, .ccis = 0, .cm = 0 };
  msp430_compare_control_t ccRSOutmod = { 
  .ccifg = 0, .cov = 0, .out = 0, .cci = 0, .ccie = 0, 
  .outmod = 7, .cap = 0, .clld = 0, .scs = 0, .ccis = 0, .cm = 0 };

  Msp430Adc12ImplP$ControlA1$setControl(ccResetSHI);
  Msp430Adc12ImplP$ControlA1$setControl(ccSetSHI);

  Msp430Adc12ImplP$ControlA1$setControl(ccRSOutmod);
  Msp430Adc12ImplP$TimerA$setMode(MSP430TIMER_UP_MODE);
}

# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static   error_t AdcP$finishStreamRequest$postTask(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(AdcP$finishStreamRequest);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 170 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline    error_t Msp430RefVoltArbiterImplP$AdcResource$default$release(uint8_t client)
#line 170
{
#line 170
  return FAIL;
}

# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   error_t Msp430RefVoltArbiterImplP$AdcResource$release(uint8_t arg_0x1797888){
#line 110
  unsigned char result;
#line 110

#line 110
  switch (arg_0x1797888) {
#line 110
    case /*LightSensorC.Sensor.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID:
#line 110
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$release(/*LightSensorC.Sensor.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID);
#line 110
      break;
#line 110
    case /*LightSensorC.Sensor.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$1$ID:
#line 110
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$release(/*LightSensorC.Sensor.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$1$ID);
#line 110
      break;
#line 110
    default:
#line 110
      result = Msp430RefVoltArbiterImplP$AdcResource$default$release(arg_0x1797888);
#line 110
      break;
#line 110
    }
#line 110

#line 110
  return result;
#line 110
}
#line 110
# 56 "/opt/tinyos-2.x/tos/system/RoundRobinResourceQueueC.nc"
static inline   bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$isEmpty(void)
#line 56
{
  int i;

  /* atomic removed: atomic calls only */
#line 58
  {
    for (i = 0; i < sizeof /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$resQ; i++) 
      if (/*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$resQ[i] > 0) {
          unsigned char __nesc_temp = 
#line 60
          FALSE;

#line 60
          return __nesc_temp;
        }
#line 61
    {
      unsigned char __nesc_temp = 
#line 61
      TRUE;

#line 61
      return __nesc_temp;
    }
  }
}

# 43 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static   bool /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Queue$isEmpty(void){
#line 43
  unsigned char result;
#line 43

#line 43
  result = /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$isEmpty();
#line 43

#line 43
  return result;
#line 43
}
#line 43
# 47 "/opt/tinyos-2.x/tos/system/RoundRobinResourceQueueC.nc"
static inline void /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$clearEntry(uint8_t id)
#line 47
{
  /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$resQ[id / 8] &= ~(1 << id % 8);
}

#line 69
static inline   resource_client_id_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$dequeue(void)
#line 69
{
  int i;

  /* atomic removed: atomic calls only */
#line 71
  {
    for (i = /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$last + 1; ; i++) {
        if (i == 2U) {
          i = 0;
          }
#line 75
        if (/*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$isEnqueued(i)) {
            /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$clearEntry(i);
            /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$last = i;
            {
              unsigned char __nesc_temp = 
#line 78
              i;

#line 78
              return __nesc_temp;
            }
          }
#line 80
        if (i == /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$last) {
          break;
          }
      }
#line 83
    {
      unsigned char __nesc_temp = 
#line 83
      /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$NO_ENTRY;

#line 83
      return __nesc_temp;
    }
  }
}

# 60 "/opt/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static   resource_client_id_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Queue$dequeue(void){
#line 60
  unsigned char result;
#line 60

#line 60
  result = /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$dequeue();
#line 60

#line 60
  return result;
#line 60
}
#line 60
# 168 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static inline    void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$unconfigure(uint8_t id)
#line 168
{
}

# 55 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
inline static   void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$unconfigure(uint8_t arg_0x1735c18){
#line 55
    /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$unconfigure(arg_0x1735c18);
#line 55
}
#line 55
# 91 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline   void HplAdc12P$HplAdc12$setIEFlags(uint16_t mask)
#line 91
{
#line 91
  HplAdc12P$ADC12IE = mask;
}

# 95 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static   void Msp430Adc12ImplP$HplAdc12$setIEFlags(uint16_t arg_0x1673350){
#line 95
  HplAdc12P$HplAdc12$setIEFlags(arg_0x1673350);
#line 95
}
#line 95
# 73 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline   void HplAdc12P$HplAdc12$setMCtl(uint8_t i, adc12memctl_t memControl)
#line 73
{
  uint8_t *memCtlPtr = (uint8_t *)(char *)0x0080;

#line 75
  memCtlPtr += i;
  *memCtlPtr = * (uint8_t *)&memControl;
}

# 75 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static   void Msp430Adc12ImplP$HplAdc12$setMCtl(uint8_t arg_0x1674068, adc12memctl_t arg_0x16741f8){
#line 75
  HplAdc12P$HplAdc12$setMCtl(arg_0x1674068, arg_0x16741f8);
#line 75
}
#line 75
# 61 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline   void HplAdc12P$HplAdc12$setCtl1(adc12ctl1_t control1)
#line 61
{
  HplAdc12P$ADC12CTL1 = * (uint16_t *)&control1;
}

# 57 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static   void Msp430Adc12ImplP$HplAdc12$setCtl1(adc12ctl1_t arg_0x1676400){
#line 57
  HplAdc12P$HplAdc12$setCtl1(arg_0x1676400);
#line 57
}
#line 57
#line 51
inline static   void Msp430Adc12ImplP$HplAdc12$setCtl0(adc12ctl0_t arg_0x1677e98){
#line 51
  HplAdc12P$HplAdc12$setCtl0(arg_0x1677e98);
#line 51
}
#line 51
#line 63
inline static   adc12ctl0_t Msp430Adc12ImplP$HplAdc12$getCtl0(void){
#line 63
  struct __nesc_unnamed4254 result;
#line 63

#line 63
  result = HplAdc12P$HplAdc12$getCtl0();
#line 63

#line 63
  return result;
#line 63
}
#line 63
# 136 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static inline   uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ArbiterInfo$userId(void)
#line 136
{
  /* atomic removed: atomic calls only */
#line 137
  {
    unsigned char __nesc_temp = 
#line 137
    /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$resId;

#line 137
    return __nesc_temp;
  }
}

# 88 "/opt/tinyos-2.x/tos/interfaces/ArbiterInfo.nc"
inline static   uint8_t Msp430Adc12ImplP$ADCArbiterInfo$userId(void){
#line 88
  unsigned char result;
#line 88

#line 88
  result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ArbiterInfo$userId();
#line 88

#line 88
  return result;
#line 88
}
#line 88
# 162 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline   error_t Msp430Adc12ImplP$SingleChannel$configureSingle(uint8_t id, 
const msp430adc12_channel_config_t *config)
{
  error_t result = ERESERVE;

  if (!config) {
    return EINVAL;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 170
    {
      if (Msp430Adc12ImplP$state & Msp430Adc12ImplP$ADC_BUSY) 
        {
          unsigned char __nesc_temp = 
#line 172
          EBUSY;

          {
#line 172
            __nesc_atomic_end(__nesc_atomic); 
#line 172
            return __nesc_temp;
          }
        }
#line 173
      if (Msp430Adc12ImplP$ADCArbiterInfo$userId() == id) {
          adc12ctl1_t ctl1 = { 
          .adc12busy = 0, 
          .conseq = 0, 
          .adc12ssel = config->adc12ssel, 
          .adc12div = config->adc12div, 
          .issh = 0, 
          .shp = 1, 
          .shs = 0, 
          .cstartadd = 0 };

          adc12memctl_t memctl = { 
          .inch = config->inch, 
          .sref = config->sref, 
          .eos = 1 };

          adc12ctl0_t ctl0 = Msp430Adc12ImplP$HplAdc12$getCtl0();

#line 190
          ctl0.msc = 1;
          ctl0.sht0 = config->sht;
          ctl0.sht1 = config->sht;

          Msp430Adc12ImplP$state = Msp430Adc12ImplP$SINGLE_DATA;
          Msp430Adc12ImplP$HplAdc12$setCtl0(ctl0);
          Msp430Adc12ImplP$HplAdc12$setCtl1(ctl1);
          Msp430Adc12ImplP$HplAdc12$setMCtl(0, memctl);
          Msp430Adc12ImplP$HplAdc12$setIEFlags(0x01);
          result = SUCCESS;
        }
    }
#line 201
    __nesc_atomic_end(__nesc_atomic); }
  return result;
}

# 378 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline    error_t AdcP$SingleChannel$default$configureSingle(uint8_t client, 
const msp430adc12_channel_config_t *config)
#line 379
{
#line 379
  return FAIL;
}

# 84 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static   error_t AdcP$SingleChannel$configureSingle(uint8_t arg_0x15ea740, const msp430adc12_channel_config_t *arg_0x15d35b8){
#line 84
  unsigned char result;
#line 84

#line 84
  switch (arg_0x15ea740) {
#line 84
    case /*LightSensorC.Sensor.AdcReadClientC*/AdcReadClientC$0$CLIENT:
#line 84
      result = Msp430Adc12ImplP$SingleChannel$configureSingle(/*LightSensorC.Sensor.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID, arg_0x15d35b8);
#line 84
      break;
#line 84
    default:
#line 84
      result = AdcP$SingleChannel$default$configureSingle(arg_0x15ea740, arg_0x15d35b8);
#line 84
      break;
#line 84
    }
#line 84

#line 84
  return result;
#line 84
}
#line 84
# 354 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline    const msp430adc12_channel_config_t *
AdcP$Config$default$getConfiguration(uint8_t client)
{
  return &AdcP$defaultConfig;
}

# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
inline static   AdcP$Config$adc_config_t AdcP$Config$getConfiguration(uint8_t arg_0x15dba28){
#line 58
  struct __nesc_unnamed4267 const *result;
#line 58

#line 58
  switch (arg_0x15dba28) {
#line 58
    case /*LightSensorC.Sensor.AdcReadClientC*/AdcReadClientC$0$CLIENT:
#line 58
      result = LightP$AdcConfigure$getConfiguration();
#line 58
      break;
#line 58
    default:
#line 58
      result = AdcP$Config$default$getConfiguration(arg_0x15dba28);
#line 58
      break;
#line 58
    }
#line 58

#line 58
  return result;
#line 58
}
#line 58
# 87 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline error_t AdcP$configure(uint8_t client)
{
  error_t result = EINVAL;
  const msp430adc12_channel_config_t *config;

#line 91
  config = AdcP$Config$getConfiguration(client);
  if (config->inch != INPUT_CHANNEL_NONE) {
    result = AdcP$SingleChannel$configureSingle(client, config);
    }
#line 94
  return result;
}

#line 347
static inline    error_t AdcP$SingleChannel$default$getData(uint8_t client)
{
  return EINVAL;
}

# 189 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static   error_t AdcP$SingleChannel$getData(uint8_t arg_0x15ea740){
#line 189
  unsigned char result;
#line 189

#line 189
  switch (arg_0x15ea740) {
#line 189
    case /*LightSensorC.Sensor.AdcReadClientC*/AdcReadClientC$0$CLIENT:
#line 189
      result = Msp430Adc12ImplP$SingleChannel$getData(/*LightSensorC.Sensor.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID);
#line 189
      break;
#line 189
    default:
#line 189
      result = AdcP$SingleChannel$default$getData(arg_0x15ea740);
#line 189
      break;
#line 189
    }
#line 189

#line 189
  return result;
#line 189
}
#line 189
# 161 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline  bool /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$isRunning(uint8_t num)
{
  return /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_timers[num].isrunning;
}

# 81 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static  bool LightSensorM$T2$isRunning(void){
#line 81
  unsigned char result;
#line 81

#line 81
  result = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$isRunning(3U);
#line 81

#line 81
  return result;
#line 81
}
#line 81
#line 62
inline static  void LightSensorM$T2$startOneShot(uint32_t arg_0x13a81e8){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShot(3U, arg_0x13a81e8);
#line 62
}
#line 62
# 98 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static inline   void LedsP$Leds$led2Off(void)
#line 98
{
  LedsP$Led2$set();
  ;
#line 100
  ;
}

# 46 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$clr(void)
#line 46
{
  /* atomic removed: atomic calls only */
#line 46
  * (volatile uint8_t *)49U &= ~(0x01 << 6);
}

# 39 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$clr(void){
#line 39
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$clr();
#line 39
}
#line 39
# 38 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$clr(void)
#line 38
{
#line 38
  /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$clr();
}

# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void LedsP$Led2$clr(void){
#line 30
  /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$clr();
#line 30
}
#line 30
# 93 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static inline   void LedsP$Leds$led2On(void)
#line 93
{
  LedsP$Led2$clr();
  ;
#line 95
  ;
}

#line 83
static inline   void LedsP$Leds$led1Off(void)
#line 83
{
  LedsP$Led1$set();
  ;
#line 85
  ;
}

# 46 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$clr(void)
#line 46
{
  /* atomic removed: atomic calls only */
#line 46
  * (volatile uint8_t *)49U &= ~(0x01 << 5);
}

# 39 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$clr(void){
#line 39
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$clr();
#line 39
}
#line 39
# 38 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$clr(void)
#line 38
{
#line 38
  /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$clr();
}

# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void LedsP$Led1$clr(void){
#line 30
  /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$clr();
#line 30
}
#line 30
# 78 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static inline   void LedsP$Leds$led1On(void)
#line 78
{
  LedsP$Led1$clr();
  ;
#line 80
  ;
}

#line 68
static inline   void LedsP$Leds$led0Off(void)
#line 68
{
  LedsP$Led0$set();
  ;
#line 70
  ;
}

# 46 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$clr(void)
#line 46
{
  /* atomic removed: atomic calls only */
#line 46
  * (volatile uint8_t *)49U &= ~(0x01 << 4);
}

# 39 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$clr(void){
#line 39
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$clr();
#line 39
}
#line 39
# 38 "/opt/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline   void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$clr(void)
#line 38
{
#line 38
  /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$clr();
}

# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static   void LedsP$Led0$clr(void){
#line 30
  /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$clr();
#line 30
}
#line 30
# 63 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static inline   void LedsP$Leds$led0On(void)
#line 63
{
  LedsP$Led0$clr();
  ;
#line 65
  ;
}

#line 125
static inline   void LedsP$Leds$set(uint8_t val)
#line 125
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 126
    {
      if (val & LEDS_LED0) {
          LedsP$Leds$led0On();
        }
      else {
          LedsP$Leds$led0Off();
        }
      if (val & LEDS_LED1) {
          LedsP$Leds$led1On();
        }
      else {
          LedsP$Leds$led1Off();
        }
      if (val & LEDS_LED2) {
          LedsP$Leds$led2On();
        }
      else {
          LedsP$Leds$led2Off();
        }
    }
#line 145
    __nesc_atomic_end(__nesc_atomic); }
}

# 123 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
inline static   void LightSensorM$Leds$set(uint8_t arg_0x1395630){
#line 123
  LedsP$Leds$set(arg_0x1395630);
#line 123
}
#line 123
# 127 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$fired(void)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$fireTimers(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow());
}

# 72 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static  void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$fired(void){
#line 72
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$fired();
#line 72
}
#line 72
# 80 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline   /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getAlarm(void)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 82
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type __nesc_temp = 
#line 82
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_dt;

      {
#line 82
        __nesc_atomic_end(__nesc_atomic); 
#line 82
        return __nesc_temp;
      }
    }
#line 84
    __nesc_atomic_end(__nesc_atomic); }
}

# 105 "/opt/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static   /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$getAlarm(void){
#line 105
  unsigned long result;
#line 105

#line 105
  result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getAlarm();
#line 105

#line 105
  return result;
#line 105
}
#line 105
# 63 "/opt/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline  void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$runTask(void)
{
  if (/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_oneshot == FALSE) {
    /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$start(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$getAlarm(), /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_dt, FALSE);
    }
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$fired();
}

# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static   error_t Msp430RefVoltArbiterImplP$switchOff$postTask(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(Msp430RefVoltArbiterImplP$switchOff);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 156 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$stop(uint8_t num)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_timers[num].isrunning = FALSE;
}

# 67 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static  void Msp430RefVoltGeneratorP$SwitchOnTimer$stop(void){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$stop(0U);
#line 67
}
#line 67
# 127 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline  error_t Msp430RefVoltGeneratorP$RefVolt_1_5V$stop(void)
{
  switch (Msp430RefVoltGeneratorP$state) 
    {
      case Msp430RefVoltGeneratorP$REFERENCE_1_5V_PENDING: 

        case Msp430RefVoltGeneratorP$REFERENCE_2_5V_PENDING: 
          if (Msp430RefVoltGeneratorP$switchOff() == SUCCESS) {
              Msp430RefVoltGeneratorP$SwitchOnTimer$stop();
              Msp430RefVoltGeneratorP$state = Msp430RefVoltGeneratorP$GENERATOR_OFF;
              if (Msp430RefVoltGeneratorP$state == Msp430RefVoltGeneratorP$REFERENCE_1_5V_PENDING) {
                Msp430RefVoltGeneratorP$RefVolt_1_5V$stopDone(SUCCESS);
                }
              else {
#line 140
                Msp430RefVoltGeneratorP$RefVolt_2_5V$stopDone(SUCCESS);
                }
#line 141
              return SUCCESS;
            }
          else {
#line 143
            return FAIL;
            }
#line 144
      case Msp430RefVoltGeneratorP$REFERENCE_1_5V_STABLE: 

        case Msp430RefVoltGeneratorP$REFERENCE_2_5V_STABLE: 
          Msp430RefVoltGeneratorP$SwitchOffTimer$startOneShot(20);
      return SUCCESS;
      case Msp430RefVoltGeneratorP$GENERATOR_OFF: 

        default: 

          return FAIL;
    }
}

# 109 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static  error_t Msp430RefVoltArbiterImplP$RefVolt_1_5V$stop(void){
#line 109
  unsigned char result;
#line 109

#line 109
  result = Msp430RefVoltGeneratorP$RefVolt_1_5V$stop();
#line 109

#line 109
  return result;
#line 109
}
#line 109
# 136 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline  void Msp430RefVoltArbiterImplP$switchOff$runTask(void)
{

  if (Msp430RefVoltArbiterImplP$syncOwner != Msp430RefVoltArbiterImplP$NO_OWNER) {
      if (Msp430RefVoltArbiterImplP$RefVolt_1_5V$stop() == SUCCESS) {
          Msp430RefVoltArbiterImplP$syncOwner = Msp430RefVoltArbiterImplP$NO_OWNER;
        }
      else {
#line 143
        Msp430RefVoltArbiterImplP$switchOff$postTask();
        }
    }
}

# 160 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static inline   void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$default$granted(uint8_t id)
#line 160
{
}

# 92 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static  void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$granted(uint8_t arg_0x1737830){
#line 92
  switch (arg_0x1737830) {
#line 92
    case /*LightSensorC.Sensor.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID:
#line 92
      Msp430RefVoltArbiterImplP$AdcResource$granted(/*LightSensorC.Sensor.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID);
#line 92
      break;
#line 92
    case /*LightSensorC.Sensor.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$1$ID:
#line 92
      Msp430RefVoltArbiterImplP$AdcResource$granted(/*LightSensorC.Sensor.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$1$ID);
#line 92
      break;
#line 92
    default:
#line 92
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$default$granted(arg_0x1737830);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 166 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static inline    void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$configure(uint8_t id)
#line 166
{
}

# 49 "/opt/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
inline static   void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$configure(uint8_t arg_0x1735c18){
#line 49
    /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$configure(arg_0x1735c18);
#line 49
}
#line 49
# 150 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static inline  void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask$runTask(void)
#line 150
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 151
    {
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$resId = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$reqResId;
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$state = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$RES_BUSY;
    }
#line 154
    __nesc_atomic_end(__nesc_atomic); }
  /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$configure(/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$resId);
  /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$granted(/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$resId);
}

# 172 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline    const msp430adc12_channel_config_t *
Msp430RefVoltArbiterImplP$Config$default$getConfiguration(uint8_t client)
{
  return &Msp430RefVoltArbiterImplP$defaultConfig;
}

# 58 "/opt/tinyos-2.x/tos/interfaces/AdcConfigure.nc"
inline static   Msp430RefVoltArbiterImplP$Config$adc_config_t Msp430RefVoltArbiterImplP$Config$getConfiguration(uint8_t arg_0x1795e58){
#line 58
  struct __nesc_unnamed4267 const *result;
#line 58

#line 58
  switch (arg_0x1795e58) {
#line 58
    case /*LightSensorC.Sensor.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID:
#line 58
      result = LightP$AdcConfigure$getConfiguration();
#line 58
      break;
#line 58
    case /*LightSensorC.Sensor.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$1$ID:
#line 58
      result = LightP$AdcConfigure$getConfiguration();
#line 58
      break;
#line 58
    default:
#line 58
      result = Msp430RefVoltArbiterImplP$Config$default$getConfiguration(arg_0x1795e58);
#line 58
      break;
#line 58
    }
#line 58

#line 58
  return result;
#line 58
}
#line 58
# 67 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static  void Msp430RefVoltGeneratorP$SwitchOffTimer$stop(void){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$stop(1U);
#line 67
}
#line 67
#line 62
inline static  void Msp430RefVoltGeneratorP$SwitchOnTimer$startOneShot(uint32_t arg_0x13a81e8){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShot(0U, arg_0x13a81e8);
#line 62
}
#line 62
# 94 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline  error_t Msp430RefVoltGeneratorP$RefVolt_1_5V$start(void)
{
  switch (Msp430RefVoltGeneratorP$state) 
    {
      case Msp430RefVoltGeneratorP$REFERENCE_1_5V_STABLE: 
        Msp430RefVoltGeneratorP$SwitchOffTimer$stop();
      Msp430RefVoltGeneratorP$RefVolt_1_5V$startDone(SUCCESS);
      return SUCCESS;
      case Msp430RefVoltGeneratorP$GENERATOR_OFF: 
        if (Msp430RefVoltGeneratorP$switchOn(Msp430RefVoltGeneratorP$REFERENCE_1_5V_PENDING) == SUCCESS) {
            Msp430RefVoltGeneratorP$SwitchOnTimer$startOneShot(17);
            Msp430RefVoltGeneratorP$state = Msp430RefVoltGeneratorP$REFERENCE_1_5V_PENDING;
            return SUCCESS;
          }
        else {
#line 108
          return FAIL;
          }
#line 109
      case Msp430RefVoltGeneratorP$REFERENCE_2_5V_STABLE: 
        if (Msp430RefVoltGeneratorP$switchOn(Msp430RefVoltGeneratorP$REFERENCE_1_5V_PENDING) == SUCCESS) {
            Msp430RefVoltGeneratorP$SwitchOffTimer$stop();
            Msp430RefVoltGeneratorP$state = Msp430RefVoltGeneratorP$REFERENCE_1_5V_STABLE;
            Msp430RefVoltGeneratorP$RefVolt_1_5V$startDone(SUCCESS);
            return SUCCESS;
          }
        else {
#line 116
          return FAIL;
          }
#line 117
      case Msp430RefVoltGeneratorP$REFERENCE_1_5V_PENDING: 

        case Msp430RefVoltGeneratorP$REFERENCE_2_5V_PENDING: 

          default: 

            return FAIL;
    }
}

# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static  error_t Msp430RefVoltArbiterImplP$RefVolt_1_5V$start(void){
#line 83
  unsigned char result;
#line 83

#line 83
  result = Msp430RefVoltGeneratorP$RefVolt_1_5V$start();
#line 83

#line 83
  return result;
#line 83
}
#line 83
# 157 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline  error_t Msp430RefVoltGeneratorP$RefVolt_2_5V$start(void)
{
  switch (Msp430RefVoltGeneratorP$state) 
    {
      case Msp430RefVoltGeneratorP$REFERENCE_2_5V_STABLE: 
        Msp430RefVoltGeneratorP$SwitchOffTimer$stop();
      Msp430RefVoltGeneratorP$RefVolt_2_5V$startDone(SUCCESS);
      return SUCCESS;
      case Msp430RefVoltGeneratorP$GENERATOR_OFF: 
        if (Msp430RefVoltGeneratorP$switchOn(Msp430RefVoltGeneratorP$REFERENCE_2_5V_PENDING) == SUCCESS) {
            Msp430RefVoltGeneratorP$SwitchOnTimer$startOneShot(17);
            Msp430RefVoltGeneratorP$state = Msp430RefVoltGeneratorP$REFERENCE_2_5V_PENDING;
            return SUCCESS;
          }
        else {
#line 171
          return FAIL;
          }
#line 172
      case Msp430RefVoltGeneratorP$REFERENCE_1_5V_STABLE: 
        if (Msp430RefVoltGeneratorP$switchOn(Msp430RefVoltGeneratorP$REFERENCE_2_5V_PENDING) == SUCCESS) {
            Msp430RefVoltGeneratorP$SwitchOffTimer$stop();
            Msp430RefVoltGeneratorP$state = Msp430RefVoltGeneratorP$REFERENCE_2_5V_STABLE;
            Msp430RefVoltGeneratorP$RefVolt_2_5V$startDone(SUCCESS);
            return SUCCESS;
          }
        else {
#line 179
          return FAIL;
          }
#line 180
      case Msp430RefVoltGeneratorP$REFERENCE_2_5V_PENDING: 

        case Msp430RefVoltGeneratorP$REFERENCE_1_5V_PENDING: 

          default: 

            return FAIL;
    }
}

# 83 "/opt/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static  error_t Msp430RefVoltArbiterImplP$RefVolt_2_5V$start(void){
#line 83
  unsigned char result;
#line 83

#line 83
  result = Msp430RefVoltGeneratorP$RefVolt_2_5V$start();
#line 83

#line 83
  return result;
#line 83
}
#line 83
# 328 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline   void AdcP$Read$default$readDone(uint8_t client, error_t result, uint16_t val)
#line 328
{
}

# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
inline static  void AdcP$Read$readDone(uint8_t arg_0x15b2010, error_t arg_0x13b47f0, AdcP$Read$val_t arg_0x13b4970){
#line 63
  switch (arg_0x15b2010) {
#line 63
    case /*LightSensorC.Sensor.AdcReadClientC*/AdcReadClientC$0$CLIENT:
#line 63
      LightSensorM$Read$readDone(arg_0x13b47f0, arg_0x13b4970);
#line 63
      break;
#line 63
    default:
#line 63
      AdcP$Read$default$readDone(arg_0x15b2010, arg_0x13b47f0, arg_0x13b4970);
#line 63
      break;
#line 63
    }
#line 63
}
#line 63
# 326 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline    error_t AdcP$ResourceRead$default$release(uint8_t client)
#line 326
{
#line 326
  return FAIL;
}

# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   error_t AdcP$ResourceRead$release(uint8_t arg_0x15c0660){
#line 110
  unsigned char result;
#line 110

#line 110
  switch (arg_0x15c0660) {
#line 110
    case /*LightSensorC.Sensor.AdcReadClientC*/AdcReadClientC$0$CLIENT:
#line 110
      result = Msp430RefVoltArbiterImplP$ClientResource$release(/*LightSensorC.Sensor.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID);
#line 110
      break;
#line 110
    default:
#line 110
      result = AdcP$ResourceRead$default$release(arg_0x15c0660);
#line 110
      break;
#line 110
    }
#line 110

#line 110
  return result;
#line 110
}
#line 110
# 161 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline void  AdcP$readDone$runTask(void)
{
  AdcP$ResourceRead$release(AdcP$owner);
  AdcP$Read$readDone(AdcP$owner, SUCCESS, AdcP$value);
}

#line 343
static inline   void AdcP$ReadStream$default$bufferDone(uint8_t streamClient, error_t result, 
uint16_t *buf, uint16_t count)
#line 344
{
}

# 89 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
inline static  void AdcP$ReadStream$bufferDone(uint8_t arg_0x15c61a0, error_t arg_0x1595718, AdcP$ReadStream$val_t *arg_0x15958c8, uint16_t arg_0x1595a50){
#line 89
    AdcP$ReadStream$default$bufferDone(arg_0x15c61a0, arg_0x1595718, arg_0x15958c8, arg_0x1595a50);
#line 89
}
#line 89
# 312 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline void  AdcP$signalBufferDone$runTask(void)
{
  AdcP$ReadStream$bufferDone(AdcP$owner, SUCCESS, AdcP$resultBuf, AdcP$value);
  AdcP$resultBuf = 0;
}

#line 345
static inline   void AdcP$ReadStream$default$readDone(uint8_t streamClient, error_t result, uint32_t actualPeriod)
#line 345
{
}

# 102 "/opt/tinyos-2.x/tos/interfaces/ReadStream.nc"
inline static  void AdcP$ReadStream$readDone(uint8_t arg_0x15c61a0, error_t arg_0x15940b0, uint32_t arg_0x1594240){
#line 102
    AdcP$ReadStream$default$readDone(arg_0x15c61a0, arg_0x15940b0, arg_0x1594240);
#line 102
}
#line 102
# 341 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline    error_t AdcP$ResourceReadStream$default$release(uint8_t streamClient)
#line 341
{
#line 341
  return FAIL;
}

# 110 "/opt/tinyos-2.x/tos/interfaces/Resource.nc"
inline static   error_t AdcP$ResourceReadStream$release(uint8_t arg_0x15d6848){
#line 110
  unsigned char result;
#line 110

#line 110
  switch (arg_0x15d6848) {
#line 110
    case /*LightSensorC.Sensor.AdcReadStreamClientC*/AdcReadStreamClientC$0$RSCLIENT:
#line 110
      result = Msp430RefVoltArbiterImplP$ClientResource$release(/*LightSensorC.Sensor.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$1$ID);
#line 110
      break;
#line 110
    default:
#line 110
      result = AdcP$ResourceReadStream$default$release(arg_0x15d6848);
#line 110
      break;
#line 110
    }
#line 110

#line 110
  return result;
#line 110
}
#line 110
# 222 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline void  AdcP$finishStreamRequest$runTask(void)
{
  AdcP$ResourceReadStream$release(AdcP$owner);
  if (!AdcP$streamBuf[AdcP$owner]) {

    AdcP$ReadStream$readDone(AdcP$owner, SUCCESS, AdcP$usPeriod[AdcP$owner]);
    }
  else 
#line 228
    {








      AdcP$ReadStream$readDone(AdcP$owner, FAIL, 0);
    }
}

# 49 "/opt/tinyos-2.x/tos/types/TinyError.h"
static inline error_t ecombine(error_t r1, error_t r2)




{
  return r1 == r2 ? r1 : FAIL;
}

# 111 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline   void HplAdc12P$HplAdc12$stopConversion(void)
#line 111
{
  HplAdc12P$ADC12CTL0 &= ~(0x0001 + 0x0002);
  HplAdc12P$ADC12CTL0 &= ~0x0010;
}

# 123 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static   void Msp430Adc12ImplP$HplAdc12$stopConversion(void){
#line 123
  HplAdc12P$HplAdc12$stopConversion();
#line 123
}
#line 123
# 87 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline  error_t Msp430Adc12ImplP$Init$init(void)
{
  Msp430Adc12ImplP$HplAdc12$stopConversion();
  return SUCCESS;
}

# 51 "/opt/tinyos-2.x/tos/system/RoundRobinResourceQueueC.nc"
static inline  error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$Init$init(void)
#line 51
{
  memset(/*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$resQ, 0, sizeof /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$resQ);
  return SUCCESS;
}

# 46 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t x)
#line 46
{
#line 46
  union  {
#line 46
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t f;
#line 46
    uint16_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$compareControl(void)
{
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$CC2int(x);
}

#line 94
static inline   void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$setControlAsCompare(void)
{
  * (volatile uint16_t *)386U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$compareControl();
}

# 36 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static   void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$setControlAsCompare(void){
#line 36
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$setControlAsCompare();
#line 36
}
#line 36
# 42 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline  error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Init$init(void)
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$setControlAsCompare();
  return SUCCESS;
}

# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
inline static  error_t RealMainP$SoftwareInit$init(void){
#line 51
  unsigned char result;
#line 51

#line 51
  result = /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Init$init();
#line 51
  result = ecombine(result, /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$Init$init());
#line 51
  result = ecombine(result, Msp430Adc12ImplP$Init$init());
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 146 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startPeriodic(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow(), dt, FALSE);
}

# 53 "/opt/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static  void LightSensorM$T1$startPeriodic(uint32_t arg_0x1390c10){
#line 53
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startPeriodic(2U, arg_0x1390c10);
#line 53
}
#line 53
# 14 "LightSensorM.nc"
static inline  void LightSensorM$Boot$booted(void)
#line 14
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 15
    {
      P2DIR |= 1 << 1;
      P2OUT |= 1 << 1;
    }
#line 18
    __nesc_atomic_end(__nesc_atomic); }
  LightSensorM$T1$startPeriodic(10);
}

# 49 "/opt/tinyos-2.x/tos/interfaces/Boot.nc"
inline static  void RealMainP$Boot$booted(void){
#line 49
  LightSensorM$Boot$booted();
#line 49
}
#line 49
# 190 "/opt/tinyos-2.x/tos/chips/msp430/msp430hardware.h"
static inline void __nesc_disable_interrupt(void )
{
   __asm volatile ("dint");
   __asm volatile ("nop");}

# 124 "/opt/tinyos-2.x/tos/chips/msp430/McuSleepC.nc"
static inline    mcu_power_t McuSleepC$McuPowerOverride$default$lowestState(void)
#line 124
{
  return MSP430_POWER_LPM4;
}

# 54 "/opt/tinyos-2.x/tos/interfaces/McuPowerOverride.nc"
inline static   mcu_power_t McuSleepC$McuPowerOverride$lowestState(void){
#line 54
  unsigned char result;
#line 54

#line 54
  result = McuSleepC$McuPowerOverride$default$lowestState();
#line 54

#line 54
  return result;
#line 54
}
#line 54
# 66 "/opt/tinyos-2.x/tos/chips/msp430/McuSleepC.nc"
static inline mcu_power_t McuSleepC$getPowerState(void)
#line 66
{
  mcu_power_t pState = MSP430_POWER_LPM3;





  if (((((
#line 69
  TA0CCTL0 & 0x0010 || 
  TA0CCTL1 & 0x0010) || 
  TA0CCTL2 & 0x0010) && (
  TA0CTL & (3 << 8)) == 2 << 8) || (
  ME1 & ((1 << 7) | (1 << 6)) && U0TCTL & 0x20)) || (
  ME2 & ((1 << 5) | (1 << 4)) && U1TCTL & 0x20)) {






    pState = MSP430_POWER_LPM1;
    }


  if (ADC12CTL0 & 0x0010) {
      if (ADC12CTL1 & (2 << 3)) {

          if (ADC12CTL1 & (1 << 3)) {
            pState = MSP430_POWER_LPM1;
            }
          else {
#line 91
            pState = MSP430_POWER_ACTIVE;
            }
        }
      else {
#line 92
        if (ADC12CTL1 & 0x0400 && (TA0CTL & (3 << 8)) == 2 << 8) {



            pState = MSP430_POWER_LPM1;
          }
        }
    }

  return pState;
}

# 178 "/opt/tinyos-2.x/tos/chips/msp430/msp430hardware.h"
static inline mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)
#line 178
{
  return m1 < m2 ? m1 : m2;
}

# 104 "/opt/tinyos-2.x/tos/chips/msp430/McuSleepC.nc"
static inline void McuSleepC$computePowerState(void)
#line 104
{
  McuSleepC$powerState = mcombine(McuSleepC$getPowerState(), 
  McuSleepC$McuPowerOverride$lowestState());
}

static inline   void McuSleepC$McuSleep$sleep(void)
#line 109
{
  uint16_t temp;

#line 111
  if (McuSleepC$dirty) {
      McuSleepC$computePowerState();
    }

  temp = McuSleepC$msp430PowerBits[McuSleepC$powerState] | 0x0008;
   __asm volatile ("bis  %0, r2" :  : "m"(temp));
  __nesc_disable_interrupt();
}

# 59 "/opt/tinyos-2.x/tos/interfaces/McuSleep.nc"
inline static   void SchedulerBasicP$McuSleep$sleep(void){
#line 59
  McuSleepC$McuSleep$sleep();
#line 59
}
#line 59
# 67 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static __inline uint8_t SchedulerBasicP$popTask(void)
{
  if (SchedulerBasicP$m_head != SchedulerBasicP$NO_TASK) 
    {
      uint8_t id = SchedulerBasicP$m_head;

#line 72
      SchedulerBasicP$m_head = SchedulerBasicP$m_next[SchedulerBasicP$m_head];
      if (SchedulerBasicP$m_head == SchedulerBasicP$NO_TASK) 
        {
          SchedulerBasicP$m_tail = SchedulerBasicP$NO_TASK;
        }
      SchedulerBasicP$m_next[id] = SchedulerBasicP$NO_TASK;
      return id;
    }
  else 
    {
      return SchedulerBasicP$NO_TASK;
    }
}

#line 138
static inline  void SchedulerBasicP$Scheduler$taskLoop(void)
{
  for (; ; ) 
    {
      uint8_t nextTask;

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        {
          while ((nextTask = SchedulerBasicP$popTask()) == SchedulerBasicP$NO_TASK) 
            {
              SchedulerBasicP$McuSleep$sleep();
            }
        }
#line 150
        __nesc_atomic_end(__nesc_atomic); }
      SchedulerBasicP$TaskBasic$runTask(nextTask);
    }
}

# 61 "/opt/tinyos-2.x/tos/interfaces/Scheduler.nc"
inline static  void RealMainP$Scheduler$taskLoop(void){
#line 61
  SchedulerBasicP$Scheduler$taskLoop();
#line 61
}
#line 61
# 186 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline   uint16_t *AdcP$SingleChannel$multipleDataReady(uint8_t client, 
uint16_t *buf, uint16_t length)
{

  return 0;
}

# 602 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline    uint16_t *Msp430Adc12ImplP$SingleChannel$default$multipleDataReady(uint8_t id, 
uint16_t *buf, uint16_t length)
{
  return 0;
}

# 227 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static   uint16_t *Msp430Adc12ImplP$SingleChannel$multipleDataReady(uint8_t arg_0x1666508, uint16_t arg_0x15eb9d0[], uint16_t arg_0x15ebb60){
#line 227
  unsigned int *result;
#line 227

#line 227
  switch (arg_0x1666508) {
#line 227
    case /*LightSensorC.Sensor.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID:
#line 227
      result = AdcP$SingleChannel$multipleDataReady(/*LightSensorC.Sensor.AdcReadClientC*/AdcReadClientC$0$CLIENT, arg_0x15eb9d0, arg_0x15ebb60);
#line 227
      break;
#line 227
    case /*LightSensorC.Sensor.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$1$ID:
#line 227
      result = AdcP$SingleChannelReadStream$multipleDataReady(/*LightSensorC.Sensor.AdcReadStreamClientC*/AdcReadStreamClientC$0$RSCLIENT, arg_0x15eb9d0, arg_0x15ebb60);
#line 227
      break;
#line 227
    default:
#line 227
      result = Msp430Adc12ImplP$SingleChannel$default$multipleDataReady(arg_0x1666508, arg_0x15eb9d0, arg_0x15ebb60);
#line 227
      break;
#line 227
    }
#line 227

#line 227
  return result;
#line 227
}
#line 227
# 87 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline   uint16_t HplAdc12P$HplAdc12$getMem(uint8_t i)
#line 87
{
  return *((uint16_t *)(int *)0x0140 + i);
}

# 89 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static   uint16_t Msp430Adc12ImplP$HplAdc12$getMem(uint8_t arg_0x1674d40){
#line 89
  unsigned int result;
#line 89

#line 89
  result = HplAdc12P$HplAdc12$getMem(arg_0x1674d40);
#line 89

#line 89
  return result;
#line 89
}
#line 89
#line 82
inline static   adc12memctl_t Msp430Adc12ImplP$HplAdc12$getMCtl(uint8_t arg_0x16747a0){
#line 82
  struct __nesc_unnamed4268 result;
#line 82

#line 82
  result = HplAdc12P$HplAdc12$getMCtl(arg_0x16747a0);
#line 82

#line 82
  return result;
#line 82
}
#line 82
# 116 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline   void HplAdc12P$HplAdc12$enableConversion(void)
#line 116
{
  HplAdc12P$ADC12CTL0 |= 0x0002;
}

# 133 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static   void Msp430Adc12ImplP$HplAdc12$enableConversion(void){
#line 133
  HplAdc12P$HplAdc12$enableConversion();
#line 133
}
#line 133
# 608 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline    void Msp430Adc12ImplP$MultiChannel$default$dataReady(uint8_t id, uint16_t *buffer, uint16_t numSamples)
#line 608
{
}

# 110 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc"
inline static   void Msp430Adc12ImplP$MultiChannel$dataReady(uint8_t arg_0x1664010, uint16_t *arg_0x1658e80, uint16_t arg_0x1656030){
#line 110
    Msp430Adc12ImplP$MultiChannel$default$dataReady(arg_0x1664010, arg_0x1658e80, arg_0x1656030);
#line 110
}
#line 110
# 318 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline   error_t AdcP$SingleChannelReadStream$singleDataReady(uint8_t streamClient, uint16_t data)
{

  return SUCCESS;
}

# 597 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline    error_t Msp430Adc12ImplP$SingleChannel$default$singleDataReady(uint8_t id, uint16_t data)
{
  return FAIL;
}

# 206 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static   error_t Msp430Adc12ImplP$SingleChannel$singleDataReady(uint8_t arg_0x1666508, uint16_t arg_0x15eb230){
#line 206
  unsigned char result;
#line 206

#line 206
  switch (arg_0x1666508) {
#line 206
    case /*LightSensorC.Sensor.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID:
#line 206
      result = AdcP$SingleChannel$singleDataReady(/*LightSensorC.Sensor.AdcReadClientC*/AdcReadClientC$0$CLIENT, arg_0x15eb230);
#line 206
      break;
#line 206
    case /*LightSensorC.Sensor.AdcReadStreamClientC.Msp430AdcPlient*/Msp430Adc12ClientAutoRVGC$1$ID:
#line 206
      result = AdcP$SingleChannelReadStream$singleDataReady(/*LightSensorC.Sensor.AdcReadStreamClientC*/AdcReadStreamClientC$0$RSCLIENT, arg_0x15eb230);
#line 206
      break;
#line 206
    default:
#line 206
      result = Msp430Adc12ImplP$SingleChannel$default$singleDataReady(arg_0x1666508, arg_0x15eb230);
#line 206
      break;
#line 206
    }
#line 206

#line 206
  return result;
#line 206
}
#line 206
# 611 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline    void Msp430Adc12ImplP$Overflow$default$conversionTimeOverflow(uint8_t id)
#line 611
{
}

# 54 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
inline static   void Msp430Adc12ImplP$Overflow$conversionTimeOverflow(uint8_t arg_0x1664798){
#line 54
    Msp430Adc12ImplP$Overflow$default$conversionTimeOverflow(arg_0x1664798);
#line 54
}
#line 54
# 610 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline    void Msp430Adc12ImplP$Overflow$default$memOverflow(uint8_t id)
#line 610
{
}

# 49 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
inline static   void Msp430Adc12ImplP$Overflow$memOverflow(uint8_t arg_0x1664798){
#line 49
    Msp430Adc12ImplP$Overflow$default$memOverflow(arg_0x1664798);
#line 49
}
#line 49
# 514 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline   void Msp430Adc12ImplP$HplAdc12$conversionDone(uint16_t iv)
{
  if (iv <= 4) {
      if (iv == 2) {
        Msp430Adc12ImplP$Overflow$memOverflow(Msp430Adc12ImplP$clientID);
        }
      else {
#line 520
        Msp430Adc12ImplP$Overflow$conversionTimeOverflow(Msp430Adc12ImplP$clientID);
        }
    }
  switch (Msp430Adc12ImplP$state & Msp430Adc12ImplP$CONVERSION_MODE_MASK) 
    {
      case Msp430Adc12ImplP$SINGLE_DATA: 
        Msp430Adc12ImplP$stopConversion();
      Msp430Adc12ImplP$SingleChannel$singleDataReady(Msp430Adc12ImplP$clientID, Msp430Adc12ImplP$HplAdc12$getMem(0));
      break;
      case Msp430Adc12ImplP$SINGLE_DATA_REPEAT: 
        {
          error_t repeatContinue;

#line 532
          repeatContinue = Msp430Adc12ImplP$SingleChannel$singleDataReady(Msp430Adc12ImplP$clientID, 
          Msp430Adc12ImplP$HplAdc12$getMem(0));
          if (repeatContinue == FAIL) {
            Msp430Adc12ImplP$stopConversion();
            }
#line 536
          break;
        }
      case Msp430Adc12ImplP$MULTI_CHANNEL: 
        {
          uint16_t i = 0;

#line 541
          do {
              * Msp430Adc12ImplP$resultBuffer++ = Msp430Adc12ImplP$HplAdc12$getMem(i);
            }
          while (
#line 543
          ++i < Msp430Adc12ImplP$numChannels);
          Msp430Adc12ImplP$resultBufferIndex += Msp430Adc12ImplP$numChannels;
          if (Msp430Adc12ImplP$resultBufferLength == Msp430Adc12ImplP$resultBufferIndex) {
              Msp430Adc12ImplP$stopConversion();
              Msp430Adc12ImplP$resultBuffer -= Msp430Adc12ImplP$resultBufferLength;
              Msp430Adc12ImplP$resultBufferIndex = 0;
              Msp430Adc12ImplP$MultiChannel$dataReady(Msp430Adc12ImplP$clientID, Msp430Adc12ImplP$resultBuffer, Msp430Adc12ImplP$resultBufferLength);
            }
          else {
#line 550
            Msp430Adc12ImplP$HplAdc12$enableConversion();
            }
        }
#line 552
      break;
      case Msp430Adc12ImplP$MULTIPLE_DATA: 
        {
          uint16_t i = 0;
#line 555
          uint16_t length;

#line 556
          if (Msp430Adc12ImplP$resultBufferLength - Msp430Adc12ImplP$resultBufferIndex > 16) {
            length = 16;
            }
          else {
#line 559
            length = Msp430Adc12ImplP$resultBufferLength - Msp430Adc12ImplP$resultBufferIndex;
            }
#line 560
          do {
              * Msp430Adc12ImplP$resultBuffer++ = Msp430Adc12ImplP$HplAdc12$getMem(i);
            }
          while (
#line 562
          ++i < length);
          Msp430Adc12ImplP$resultBufferIndex += length;

          if (Msp430Adc12ImplP$resultBufferLength - Msp430Adc12ImplP$resultBufferIndex > 15) {
            return;
            }
          else {
#line 567
            if (Msp430Adc12ImplP$resultBufferLength - Msp430Adc12ImplP$resultBufferIndex > 0) {
                adc12memctl_t memctl = Msp430Adc12ImplP$HplAdc12$getMCtl(0);

#line 569
                memctl.eos = 1;
                Msp430Adc12ImplP$HplAdc12$setMCtl(Msp430Adc12ImplP$resultBufferLength - Msp430Adc12ImplP$resultBufferIndex, memctl);
              }
            else 
#line 571
              {
                Msp430Adc12ImplP$stopConversion();
                Msp430Adc12ImplP$resultBuffer -= Msp430Adc12ImplP$resultBufferLength;
                Msp430Adc12ImplP$resultBufferIndex = 0;
                Msp430Adc12ImplP$SingleChannel$multipleDataReady(Msp430Adc12ImplP$clientID, Msp430Adc12ImplP$resultBuffer, Msp430Adc12ImplP$resultBufferLength);
              }
            }
        }
#line 578
      break;
      case Msp430Adc12ImplP$MULTIPLE_DATA_REPEAT: 
        {
          uint8_t i = 0;

#line 582
          do {
              * Msp430Adc12ImplP$resultBuffer++ = Msp430Adc12ImplP$HplAdc12$getMem(i);
            }
          while (
#line 584
          ++i < Msp430Adc12ImplP$resultBufferLength);

          Msp430Adc12ImplP$resultBuffer = Msp430Adc12ImplP$SingleChannel$multipleDataReady(Msp430Adc12ImplP$clientID, 
          Msp430Adc12ImplP$resultBuffer - Msp430Adc12ImplP$resultBufferLength, 
          Msp430Adc12ImplP$resultBufferLength);
          if (!Msp430Adc12ImplP$resultBuffer) {
            Msp430Adc12ImplP$stopConversion();
            }
#line 591
          break;
        }
    }
}

# 274 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline   void Msp430RefVoltGeneratorP$HplAdc12$conversionDone(uint16_t iv)
#line 274
{
}

# 112 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static   void HplAdc12P$HplAdc12$conversionDone(uint16_t arg_0x1672010){
#line 112
  Msp430RefVoltGeneratorP$HplAdc12$conversionDone(arg_0x1672010);
#line 112
  Msp430Adc12ImplP$HplAdc12$conversionDone(arg_0x1672010);
#line 112
}
#line 112
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$selectIOFunc(void)
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t *)55U &= ~(0x01 << 0);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port60$selectIOFunc(void){
#line 85
  /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$selectIOFunc(void)
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t *)55U &= ~(0x01 << 1);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port61$selectIOFunc(void){
#line 85
  /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$selectIOFunc(void)
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t *)55U &= ~(0x01 << 2);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port62$selectIOFunc(void){
#line 85
  /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$selectIOFunc(void)
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t *)55U &= ~(0x01 << 3);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port63$selectIOFunc(void){
#line 85
  /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$selectIOFunc(void)
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t *)55U &= ~(0x01 << 4);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port64$selectIOFunc(void){
#line 85
  /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$selectIOFunc(void)
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t *)55U &= ~(0x01 << 5);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port65$selectIOFunc(void){
#line 85
  /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$selectIOFunc(void)
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t *)55U &= ~(0x01 << 6);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port66$selectIOFunc(void){
#line 85
  /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline   void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$selectIOFunc(void)
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t *)55U &= ~(0x01 << 7);
}

# 85 "/opt/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static   void Msp430Adc12ImplP$Port67$selectIOFunc(void){
#line 85
  /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$selectIOFunc();
#line 85
}
#line 85
# 94 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline   void HplAdc12P$HplAdc12$resetIFGs(void)
#line 94
{
  if (!HplAdc12P$ADC12IFG) {
    return;
    }
  else 
#line 97
    {

      uint8_t i;
      volatile uint16_t tmp;

#line 101
      for (i = 0; i < 16; i++) 
        tmp = HplAdc12P$HplAdc12$getMem(i);
    }
}

# 106 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc"
inline static   void Msp430Adc12ImplP$HplAdc12$resetIFGs(void){
#line 106
  HplAdc12P$HplAdc12$resetIFGs();
#line 106
}
#line 106
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static   error_t AdcP$readDone$postTask(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(AdcP$readDone);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 334 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static inline    void AdcP$ReadNow$default$readDone(uint8_t client, error_t result, uint16_t val)
#line 334
{
}

# 65 "/opt/tinyos-2.x/tos/interfaces/ReadNow.nc"
inline static   void AdcP$ReadNow$readDone(uint8_t arg_0x15b0010, error_t arg_0x15c87c0, AdcP$ReadNow$val_t arg_0x15c8940){
#line 65
    AdcP$ReadNow$default$readDone(arg_0x15b0010, arg_0x15c87c0, arg_0x15c8940);
#line 65
}
#line 65
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static   error_t AdcP$signalBufferDone$postTask(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(AdcP$signalBufferDone);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 210 "/opt/tinyos-2.x/tos/chips/msp430/msp430hardware.h"
 __nesc_atomic_t __nesc_atomic_start(void )
{
  __nesc_atomic_t result = (({
#line 212
    uint16_t __x;

#line 212
     __asm volatile ("mov	r2, %0" : "=r"((uint16_t )__x));__x;
  }
  )
#line 212
   & 0x0008) != 0;

#line 213
  __nesc_disable_interrupt();
  return result;
}

 void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)
{
  if (reenable_interrupts) {
    __nesc_enable_interrupt();
    }
}

# 11 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
 __attribute((wakeup)) __attribute((interrupt(12))) void sig_TIMERA0_VECTOR(void)
#line 11
{
#line 11
  Msp430TimerCommonP$VectorTimerA0$fired();
}

# 169 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static   void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Event$fired(void)
{
  if (/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$captured(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$fired();
    }
}

#line 169
static   void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Event$fired(void)
{
  if (/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$captured(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$fired();
    }
}

#line 169
static   void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Event$fired(void)
{
  if (/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$captured(/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$fired();
    }
}

# 12 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
 __attribute((wakeup)) __attribute((interrupt(10))) void sig_TIMERA1_VECTOR(void)
#line 12
{
#line 12
  Msp430TimerCommonP$VectorTimerA1$fired();
}

#line 13
 __attribute((wakeup)) __attribute((interrupt(26))) void sig_TIMERB0_VECTOR(void)
#line 13
{
#line 13
  Msp430TimerCommonP$VectorTimerB0$fired();
}

# 135 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static    void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$default$fired(uint8_t n)
{
}

# 28 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static   void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$fired(uint8_t arg_0x12aa9a8){
#line 28
  switch (arg_0x12aa9a8) {
#line 28
    case 0:
#line 28
      /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Event$fired();
#line 28
      break;
#line 28
    case 1:
#line 28
      /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Event$fired();
#line 28
      break;
#line 28
    case 2:
#line 28
      /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Event$fired();
#line 28
      break;
#line 28
    case 3:
#line 28
      /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Event$fired();
#line 28
      break;
#line 28
    case 4:
#line 28
      /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Event$fired();
#line 28
      break;
#line 28
    case 5:
#line 28
      /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Event$fired();
#line 28
      break;
#line 28
    case 6:
#line 28
      /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Event$fired();
#line 28
      break;
#line 28
    case 7:
#line 28
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Overflow$fired();
#line 28
      break;
#line 28
    default:
#line 28
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$default$fired(arg_0x12aa9a8);
#line 28
      break;
#line 28
    }
#line 28
}
#line 28
# 159 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static   error_t SchedulerBasicP$TaskBasic$postTask(uint8_t id)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 161
    {
#line 161
      {
        unsigned char __nesc_temp = 
#line 161
        SchedulerBasicP$pushTask(id) ? SUCCESS : EBUSY;

        {
#line 161
          __nesc_atomic_end(__nesc_atomic); 
#line 161
          return __nesc_temp;
        }
      }
    }
#line 164
    __nesc_atomic_end(__nesc_atomic); }
}

# 96 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$set_alarm(void)
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type now = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$get();
#line 98
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type expires;
#line 98
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type remaining;




  expires = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_dt;


  remaining = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type )(expires - now);


  if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0 <= now) 
    {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 121
  if (remaining > /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$MAX_DELAY) 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0 = now + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$MAX_DELAY;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_dt = remaining - /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$MAX_DELAY;
      remaining = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$MAX_DELAY;
    }
  else 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0 += /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_dt = 0;
    }
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$startAt((/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$from_size_type )now << 5, 
  (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$from_size_type )remaining << 5);
}

# 69 "/opt/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
static   /*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$get(void)
{
  /*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type rv = 0;

#line 72
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*CounterMilli32C.Transform*/TransformCounterC$0$upper_count_type high = /*CounterMilli32C.Transform*/TransformCounterC$0$m_upper;
      /*CounterMilli32C.Transform*/TransformCounterC$0$from_size_type low = /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$get();

#line 76
      if (/*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$isOverflowPending()) 
        {






          high++;
          low = /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$get();
        }
      {
        /*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type high_to = high;
        /*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type low_to = low >> /*CounterMilli32C.Transform*/TransformCounterC$0$LOW_SHIFT_RIGHT;

#line 90
        rv = (high_to << /*CounterMilli32C.Transform*/TransformCounterC$0$HIGH_SHIFT_LEFT) | low_to;
      }
    }
#line 92
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
}

# 51 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static   uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$get(void)
{




  if (1) {
      /* atomic removed: atomic calls only */
#line 58
      {
        uint16_t t0;
        uint16_t t1 = * (volatile uint16_t *)400U;

#line 61
        do {
#line 61
            t0 = t1;
#line 61
            t1 = * (volatile uint16_t *)400U;
          }
        while (
#line 61
        t0 != t1);
        {
          unsigned int __nesc_temp = 
#line 62
          t1;

#line 62
          return __nesc_temp;
        }
      }
    }
  else 
#line 65
    {
      return * (volatile uint16_t *)400U;
    }
}

# 14 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
 __attribute((wakeup)) __attribute((interrupt(24))) void sig_TIMERB1_VECTOR(void)
#line 14
{
#line 14
  Msp430TimerCommonP$VectorTimerB1$fired();
}

# 52 "/opt/tinyos-2.x/tos/system/RealMainP.nc"
  int main(void)
#line 52
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {





      {
      }
#line 60
      ;

      RealMainP$Scheduler$init();





      RealMainP$PlatformInit$init();
      while (RealMainP$Scheduler$runNextTask()) ;





      RealMainP$SoftwareInit$init();
      while (RealMainP$Scheduler$runNextTask()) ;
    }
#line 77
    __nesc_atomic_end(__nesc_atomic); }


  __nesc_enable_interrupt();

  RealMainP$Boot$booted();


  RealMainP$Scheduler$taskLoop();




  return -1;
}

# 142 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static void Msp430ClockP$set_dco_calib(int calib)
{
  BCSCTL1 = (BCSCTL1 & ~0x07) | ((calib >> 8) & 0x07);
  DCOCTL = calib & 0xff;
}

# 16 "/opt/tinyos-2.x/tos/platforms/hmote2420/MotePlatformC.nc"
static void MotePlatformC$TOSH_FLASH_M25P_DP_bit(bool set)
#line 16
{
  if (set) {
    TOSH_SET_SIMO0_PIN();
    }
  else {
#line 20
    TOSH_CLR_SIMO0_PIN();
    }
#line 21
  TOSH_SET_UCLK0_PIN();
  TOSH_CLR_UCLK0_PIN();
}

# 123 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static  bool SchedulerBasicP$Scheduler$runNextTask(void)
{
  uint8_t nextTask;

  /* atomic removed: atomic calls only */
#line 127
  {
    nextTask = SchedulerBasicP$popTask();
    if (nextTask == SchedulerBasicP$NO_TASK) 
      {
        {
          unsigned char __nesc_temp = 
#line 131
          FALSE;

#line 131
          return __nesc_temp;
        }
      }
  }
#line 134
  SchedulerBasicP$TaskBasic$runTask(nextTask);
  return TRUE;
}

#line 164
static   void SchedulerBasicP$TaskBasic$default$runTask(uint8_t id)
{
}

# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static  void SchedulerBasicP$TaskBasic$runTask(uint8_t arg_0x11e29a8){
#line 64
  switch (arg_0x11e29a8) {
#line 64
    case AdcP$finishStreamRequest:
#line 64
      AdcP$finishStreamRequest$runTask();
#line 64
      break;
#line 64
    case AdcP$signalBufferDone:
#line 64
      AdcP$signalBufferDone$runTask();
#line 64
      break;
#line 64
    case AdcP$readDone:
#line 64
      AdcP$readDone$runTask();
#line 64
      break;
#line 64
    case /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask:
#line 64
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask$runTask();
#line 64
      break;
#line 64
    case Msp430RefVoltArbiterImplP$switchOff:
#line 64
      Msp430RefVoltArbiterImplP$switchOff$runTask();
#line 64
      break;
#line 64
    case /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired:
#line 64
      /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$runTask();
#line 64
      break;
#line 64
    case /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer:
#line 64
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$runTask();
#line 64
      break;
#line 64
    default:
#line 64
      SchedulerBasicP$TaskBasic$default$runTask(arg_0x11e29a8);
#line 64
      break;
#line 64
    }
#line 64
}
#line 64
# 62 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$fireTimers(uint32_t now)
{
  uint8_t num;

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;

          if (elapsed >= timer->dt) 
            {
              if (timer->isoneshot) {
                timer->isrunning = FALSE;
                }
              else {
#line 79
                timer->t0 += timer->dt;
                }
              /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$fired(num);
            }
        }
    }
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$postTask();
}

# 143 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static   uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$isOwner(uint8_t id)
#line 143
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 144
    {
      if (/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$resId == id) {
          unsigned char __nesc_temp = 
#line 145
          TRUE;

          {
#line 145
            __nesc_atomic_end(__nesc_atomic); 
#line 145
            return __nesc_temp;
          }
        }
      else 
#line 146
        {
          unsigned char __nesc_temp = 
#line 146
          FALSE;

          {
#line 146
            __nesc_atomic_end(__nesc_atomic); 
#line 146
            return __nesc_temp;
          }
        }
    }
#line 149
    __nesc_atomic_end(__nesc_atomic); }
}

#line 71
static   error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$request(uint8_t id)
#line 71
{
  /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceRequested$requested(/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$resId);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 73
    {
      if (/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$state == /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$RES_IDLE) {
          /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$state = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$RES_GRANTING;
          /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$reqResId = id;
          /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask$postTask();
          {
            unsigned char __nesc_temp = 
#line 78
            SUCCESS;

            {
#line 78
              __nesc_atomic_end(__nesc_atomic); 
#line 78
              return __nesc_temp;
            }
          }
        }
#line 80
      {
        unsigned char __nesc_temp = 
#line 80
        /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Queue$enqueue(id);

        {
#line 80
          __nesc_atomic_end(__nesc_atomic); 
#line 80
          return __nesc_temp;
        }
      }
    }
#line 83
    __nesc_atomic_end(__nesc_atomic); }
}

# 65 "/opt/tinyos-2.x/tos/system/RoundRobinResourceQueueC.nc"
static   bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$isEnqueued(resource_client_id_t id)
#line 65
{
  return /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$resQ[id / 8] & (1 << id % 8);
}

# 78 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static error_t Msp430RefVoltGeneratorP$switchOff(void)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 80
    {
      if (Msp430RefVoltGeneratorP$HplAdc12$isBusy()) 
        {
          unsigned char __nesc_temp = 
#line 82
          FAIL;

          {
#line 82
            __nesc_atomic_end(__nesc_atomic); 
#line 82
            return __nesc_temp;
          }
        }
      else 
#line 83
        {
          adc12ctl0_t ctl0 = Msp430RefVoltGeneratorP$HplAdc12$getCtl0();

#line 85
          ctl0.enc = 0;
          Msp430RefVoltGeneratorP$HplAdc12$setCtl0(ctl0);
          ctl0.refon = 0;
          Msp430RefVoltGeneratorP$HplAdc12$setCtl0(ctl0);
          {
            unsigned char __nesc_temp = 
#line 89
            SUCCESS;

            {
#line 89
              __nesc_atomic_end(__nesc_atomic); 
#line 89
              return __nesc_temp;
            }
          }
        }
    }
#line 93
    __nesc_atomic_end(__nesc_atomic); }
}

# 151 "/opt/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static  void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShot(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow(), dt, TRUE);
}

#line 132
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_timers[num];

#line 135
  timer->t0 = t0;



  timer->dt = dt;

  timer->isoneshot = isoneshot;
  timer->isrunning = TRUE;
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$postTask();
}

# 241 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static  void AdcP$ResourceReadStream$granted(uint8_t streamClient)
{
  error_t result;
  const msp430adc12_channel_config_t *config;
  struct AdcP$stream_entry_t *entry = AdcP$streamBuf[streamClient];

  if (!entry) {
    result = EINVAL;
    }
  else 
#line 249
    {
      config = AdcP$ConfigReadStream$getConfiguration(streamClient);
      if (config->inch == INPUT_CHANNEL_NONE) {
        result = EINVAL;
        }
      else 
#line 253
        {
          AdcP$owner = streamClient;
          AdcP$streamConfig = *config;
          AdcP$streamConfig.sampcon_ssel = SAMPCON_SOURCE_SMCLK;
          AdcP$streamConfig.sampcon_id = SAMPCON_CLOCK_DIV_1;
          AdcP$streamBuf[streamClient] = entry->next;
          result = AdcP$SingleChannelReadStream$configureMultiple(streamClient, 
          &AdcP$streamConfig, (uint16_t *)entry, entry->count, AdcP$usPeriod[streamClient]);
          if (result == SUCCESS) {
            result = AdcP$SingleChannelReadStream$getData(streamClient);
            }
          else 
#line 263
            {
              AdcP$streamBuf[streamClient] = entry;
              AdcP$finishStreamRequest$postTask();
              return;
            }
        }
    }
  if (result != SUCCESS) {
      AdcP$ResourceReadStream$release(streamClient);
      AdcP$ReadStream$readDone(streamClient, FAIL, 0);
    }
  return;
}

# 253 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static   error_t Msp430Adc12ImplP$SingleChannel$configureMultiple(uint8_t id, 
const msp430adc12_channel_config_t *config, 
uint16_t *buf, uint16_t length, uint16_t jiffies)
{
  error_t result = ERESERVE;

  if ((((!config || !buf) || !length) || jiffies == 1) || jiffies == 2) {
    return EINVAL;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 262
    {
      if (Msp430Adc12ImplP$state & Msp430Adc12ImplP$ADC_BUSY) 
        {
          unsigned char __nesc_temp = 
#line 264
          EBUSY;

          {
#line 264
            __nesc_atomic_end(__nesc_atomic); 
#line 264
            return __nesc_temp;
          }
        }
#line 265
      if (Msp430Adc12ImplP$ADCArbiterInfo$userId() == id) {
          adc12ctl1_t ctl1 = { 
          .adc12busy = 0, 
          .conseq = length > 16 ? 3 : 1, 
          .adc12ssel = config->adc12ssel, 
          .adc12div = config->adc12div, 
          .issh = 0, 
          .shp = 1, 
          .shs = jiffies == 0 ? 0 : 1, 
          .cstartadd = 0 };

          adc12memctl_t memctl = { 
          .inch = config->inch, 
          .sref = config->sref, 
          .eos = 0 };

          uint16_t i;
#line 281
          uint16_t mask = 1;
          adc12ctl0_t ctl0 = Msp430Adc12ImplP$HplAdc12$getCtl0();

#line 283
          ctl0.msc = jiffies == 0 ? 1 : 0;
          ctl0.sht0 = config->sht;
          ctl0.sht1 = config->sht;

          Msp430Adc12ImplP$state = Msp430Adc12ImplP$MULTIPLE_DATA;
          Msp430Adc12ImplP$resultBuffer = buf;
          Msp430Adc12ImplP$resultBufferLength = length;
          Msp430Adc12ImplP$resultBufferIndex = 0;
          Msp430Adc12ImplP$HplAdc12$setCtl0(ctl0);
          Msp430Adc12ImplP$HplAdc12$setCtl1(ctl1);
          for (i = 0; i < length - 1 && i < 15; i++) 
            Msp430Adc12ImplP$HplAdc12$setMCtl(i, memctl);
          memctl.eos = 1;
          Msp430Adc12ImplP$HplAdc12$setMCtl(i, memctl);
          Msp430Adc12ImplP$HplAdc12$setIEFlags(mask << i);

          if (jiffies) {
              Msp430Adc12ImplP$state |= Msp430Adc12ImplP$USE_TIMERA;
              Msp430Adc12ImplP$prepareTimerA(jiffies, config->sampcon_ssel, config->sampcon_id);
            }
          result = SUCCESS;
        }
    }
#line 305
    __nesc_atomic_end(__nesc_atomic); }
  return result;
}

# 80 "/opt/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static   void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setMode(int mode)
{
  * (volatile uint16_t *)352U = (* (volatile uint16_t *)352U & ~(0x0020 | 0x0010)) | ((mode << 4) & (0x0020 | 0x0010));
}

# 366 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static   error_t Msp430Adc12ImplP$SingleChannel$getData(uint8_t id)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 368
    {
      if (Msp430Adc12ImplP$ADCArbiterInfo$userId() == id) {
          if (Msp430Adc12ImplP$state & Msp430Adc12ImplP$MULTIPLE_DATA_REPEAT && !Msp430Adc12ImplP$resultBuffer) 
            {
              unsigned char __nesc_temp = 
#line 371
              EINVAL;

              {
#line 371
                __nesc_atomic_end(__nesc_atomic); 
#line 371
                return __nesc_temp;
              }
            }
#line 372
          if (Msp430Adc12ImplP$state & Msp430Adc12ImplP$ADC_BUSY) 
            {
              unsigned char __nesc_temp = 
#line 373
              EBUSY;

              {
#line 373
                __nesc_atomic_end(__nesc_atomic); 
#line 373
                return __nesc_temp;
              }
            }
#line 374
          Msp430Adc12ImplP$state |= Msp430Adc12ImplP$ADC_BUSY;
          Msp430Adc12ImplP$clientID = id;
          Msp430Adc12ImplP$configureAdcPin(Msp430Adc12ImplP$HplAdc12$getMCtl(0).inch);
          Msp430Adc12ImplP$HplAdc12$startConversion();
          if (Msp430Adc12ImplP$state & Msp430Adc12ImplP$USE_TIMERA) {
            Msp430Adc12ImplP$startTimerA();
            }
#line 380
          {
            unsigned char __nesc_temp = 
#line 380
            SUCCESS;

            {
#line 380
              __nesc_atomic_end(__nesc_atomic); 
#line 380
              return __nesc_temp;
            }
          }
        }
    }
#line 384
    __nesc_atomic_end(__nesc_atomic); }
#line 383
  return FAIL;
}

# 79 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
static   adc12memctl_t HplAdc12P$HplAdc12$getMCtl(uint8_t i)
#line 79
{
  adc12memctl_t x = { .inch = 0, .sref = 0, .eos = 0 };
  uint8_t *memCtlPtr = (uint8_t *)(char *)0x0080;

#line 82
  memCtlPtr += i;
  x = * (adc12memctl_t *)memCtlPtr;
  return x;
}

# 116 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static   error_t Msp430RefVoltArbiterImplP$ClientResource$release(uint8_t client)
{
  error_t error;

#line 119
  if (Msp430RefVoltArbiterImplP$syncOwner == client) {
    Msp430RefVoltArbiterImplP$switchOff$postTask();
    }
#line 121
  error = Msp430RefVoltArbiterImplP$AdcResource$release(client);
#line 133
  return error;
}

# 97 "/opt/tinyos-2.x/tos/system/SimpleArbiterP.nc"
static   error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$release(uint8_t id)
#line 97
{
  bool released = FALSE;

#line 99
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 99
    {
      if (/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$state == /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$RES_BUSY && /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$resId == id) {
          if (/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Queue$isEmpty() == FALSE) {
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$reqResId = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Queue$dequeue();
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$state = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$RES_GRANTING;
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask$postTask();
            }
          else {
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$resId = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$NO_RES;
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$state = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$RES_IDLE;
            }
          released = TRUE;
        }
    }
#line 112
    __nesc_atomic_end(__nesc_atomic); }
  if (released == TRUE) {
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$unconfigure(id);
      return SUCCESS;
    }
  return FAIL;
}

# 104 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static  void AdcP$ResourceRead$granted(uint8_t client)
{

  error_t result = AdcP$configure(client);

#line 108
  if (result == SUCCESS) {
      AdcP$state = AdcP$STATE_READ;
      result = AdcP$SingleChannel$getData(client);
    }
  if (result != SUCCESS) {
      AdcP$ResourceRead$release(client);
      AdcP$Read$readDone(client, result, 0);
    }
}

# 30 "LightSensorM.nc"
static  void LightSensorM$Read$readDone(error_t result, uint16_t data)
#line 30
{
  if (result == SUCCESS) {
      if (data < 1500) {
          if (LightSensorM$T2$isRunning() == FALSE) {
            LightSensorM$T2$startOneShot(1000);
            }
        }
      else {
#line 37
        LightSensorM$Leds$set(0);
        }
    }
}

# 107 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static  void Msp430RefVoltArbiterImplP$RefVolt_2_5V$startDone(error_t error)
{
  if (Msp430RefVoltArbiterImplP$syncOwner != Msp430RefVoltArbiterImplP$NO_OWNER) {


      Msp430RefVoltArbiterImplP$ClientResource$granted(Msp430RefVoltArbiterImplP$syncOwner);
    }
}

# 136 "/opt/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static   void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0 = t0;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_dt = dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$set_alarm();
    }
#line 143
    __nesc_atomic_end(__nesc_atomic); }
}

# 70 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static  void Msp430RefVoltArbiterImplP$AdcResource$granted(uint8_t client)
{
  const msp430adc12_channel_config_t *settings = Msp430RefVoltArbiterImplP$Config$getConfiguration(client);

#line 73
  if (settings->sref == REFERENCE_VREFplus_AVss || 
  settings->sref == REFERENCE_VREFplus_VREFnegterm) {
      error_t started;

#line 76
      if (Msp430RefVoltArbiterImplP$syncOwner != Msp430RefVoltArbiterImplP$NO_OWNER) {



          Msp430RefVoltArbiterImplP$AdcResource$release(client);
          Msp430RefVoltArbiterImplP$AdcResource$request(client);
          return;
        }
      Msp430RefVoltArbiterImplP$syncOwner = client;
      if (settings->ref2_5v == REFVOLT_LEVEL_1_5) {
        started = Msp430RefVoltArbiterImplP$RefVolt_1_5V$start();
        }
      else {
#line 88
        started = Msp430RefVoltArbiterImplP$RefVolt_2_5V$start();
        }
#line 89
      if (started != SUCCESS) {
          Msp430RefVoltArbiterImplP$syncOwner = Msp430RefVoltArbiterImplP$NO_OWNER;
          Msp430RefVoltArbiterImplP$AdcResource$release(client);
          Msp430RefVoltArbiterImplP$AdcResource$request(client);
        }
    }
  else {
#line 95
    Msp430RefVoltArbiterImplP$ClientResource$granted(client);
    }
}

# 58 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static error_t Msp430RefVoltGeneratorP$switchOn(uint8_t level)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 60
    {
      if (Msp430RefVoltGeneratorP$HplAdc12$isBusy()) 
        {
          unsigned char __nesc_temp = 
#line 62
          FAIL;

          {
#line 62
            __nesc_atomic_end(__nesc_atomic); 
#line 62
            return __nesc_temp;
          }
        }
      else 
#line 63
        {
          adc12ctl0_t ctl0 = Msp430RefVoltGeneratorP$HplAdc12$getCtl0();

#line 65
          ctl0.enc = 0;
          Msp430RefVoltGeneratorP$HplAdc12$setCtl0(ctl0);
          ctl0.refon = 1;
          if (level == Msp430RefVoltGeneratorP$REFERENCE_1_5V_PENDING) {
            ctl0.r2_5v = 0;
            }
          else {
#line 71
            ctl0.r2_5v = 1;
            }
#line 72
          Msp430RefVoltGeneratorP$HplAdc12$setCtl0(ctl0);
          {
            unsigned char __nesc_temp = 
#line 73
            SUCCESS;

            {
#line 73
              __nesc_atomic_end(__nesc_atomic); 
#line 73
              return __nesc_temp;
            }
          }
        }
    }
#line 77
    __nesc_atomic_end(__nesc_atomic); }
}

# 122 "/opt/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc"
 __attribute((wakeup)) __attribute((interrupt(14))) void sig_ADC_VECTOR(void)
#line 122
{
  HplAdc12P$HplAdc12$conversionDone(HplAdc12P$ADC12IV);
}

# 471 "/opt/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static void Msp430Adc12ImplP$stopConversion(void)
{
  uint8_t i;

#line 474
  if (Msp430Adc12ImplP$state & Msp430Adc12ImplP$USE_TIMERA) {
    Msp430Adc12ImplP$TimerA$setMode(MSP430TIMER_STOP_MODE);
    }
#line 476
  Msp430Adc12ImplP$resetAdcPin(Msp430Adc12ImplP$HplAdc12$getMCtl(0).inch);
  if (Msp430Adc12ImplP$state & Msp430Adc12ImplP$MULTI_CHANNEL) {
      ADC12IV = 0;
      for (i = 1; i < Msp430Adc12ImplP$numChannels; i++) 
        Msp430Adc12ImplP$resetAdcPin(Msp430Adc12ImplP$HplAdc12$getMCtl(i).inch);
    }
  Msp430Adc12ImplP$HplAdc12$stopConversion();
  Msp430Adc12ImplP$HplAdc12$resetIFGs();
  Msp430Adc12ImplP$state &= ~Msp430Adc12ImplP$ADC_BUSY;
}

#line 145
static void Msp430Adc12ImplP$resetAdcPin(uint8_t inch)
{

  switch (inch) 
    {
      case 0: Msp430Adc12ImplP$Port60$selectIOFunc();
#line 150
      break;
      case 1: Msp430Adc12ImplP$Port61$selectIOFunc();
#line 151
      break;
      case 2: Msp430Adc12ImplP$Port62$selectIOFunc();
#line 152
      break;
      case 3: Msp430Adc12ImplP$Port63$selectIOFunc();
#line 153
      break;
      case 4: Msp430Adc12ImplP$Port64$selectIOFunc();
#line 154
      break;
      case 5: Msp430Adc12ImplP$Port65$selectIOFunc();
#line 155
      break;
      case 6: Msp430Adc12ImplP$Port66$selectIOFunc();
#line 156
      break;
      case 7: Msp430Adc12ImplP$Port67$selectIOFunc();
#line 157
      break;
    }
}

# 167 "/opt/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc"
static   error_t AdcP$SingleChannel$singleDataReady(uint8_t client, uint16_t data)
{
  switch (AdcP$state) 
    {
      case AdcP$STATE_READ: 
        AdcP$owner = client;
      AdcP$value = data;
      AdcP$readDone$postTask();
      break;
      case AdcP$STATE_READNOW: 
        AdcP$ReadNow$readDone(client, SUCCESS, data);
      break;
      default: 

        break;
    }
  return SUCCESS;
}

#line 278
static   uint16_t *AdcP$SingleChannelReadStream$multipleDataReady(uint8_t streamClient, 
uint16_t *buf, uint16_t length)
{
  error_t nextRequest;

  if (!AdcP$resultBuf) {
      AdcP$value = length;
      AdcP$resultBuf = buf;
      AdcP$signalBufferDone$postTask();
      if (!AdcP$streamBuf[streamClient]) {
        AdcP$finishStreamRequest$postTask();
        }
      else 
#line 289
        {

          struct AdcP$stream_entry_t *entry = AdcP$streamBuf[streamClient];

#line 292
          AdcP$streamBuf[streamClient] = AdcP$streamBuf[streamClient]->next;
          nextRequest = AdcP$SingleChannelReadStream$configureMultiple(streamClient, 
          &AdcP$streamConfig, (uint16_t *)entry, entry->count, AdcP$usPeriod[streamClient]);
          if (nextRequest == SUCCESS) {
            nextRequest = AdcP$SingleChannelReadStream$getData(streamClient);
            }
#line 297
          if (nextRequest != SUCCESS) {
              AdcP$streamBuf[AdcP$owner] = entry;
              AdcP$finishStreamRequest$postTask();
            }
        }
    }
  else 
#line 302
    {

      struct AdcP$stream_entry_t *entry = (struct AdcP$stream_entry_t *)buf;

#line 305
      entry->next = AdcP$streamBuf[streamClient];
      AdcP$streamBuf[streamClient] = entry;
      AdcP$finishStreamRequest$postTask();
    }
  return 0;
}

