#define nx_struct struct
#define nx_union union
#define dbg(mode, format, ...) ((void)0)
#define dbg_clear(mode, format, ...) ((void)0)
#define dbg_active(mode) 0
# 151 "/usr/lib/gcc-lib/msp430/3.2.3/include/stddef.h" 3
typedef int ptrdiff_t;
#line 213
typedef unsigned int size_t;
#line 325
typedef int wchar_t;
# 8 "/usr/lib/ncc/deputy_nodeputy.h"
struct __nesc_attr_nonnull {
}  ;
#line 9
struct __nesc_attr_bnd {
#line 9
  void *lo, *hi;
}  ;
#line 10
struct __nesc_attr_bnd_nok {
#line 10
  void *lo, *hi;
}  ;
#line 11
struct __nesc_attr_count {
#line 11
  int n;
}  ;
#line 12
struct __nesc_attr_count_nok {
#line 12
  int n;
}  ;
#line 13
struct __nesc_attr_one {
}  ;
#line 14
struct __nesc_attr_one_nok {
}  ;
#line 15
struct __nesc_attr_dmemset {
#line 15
  int a1, a2, a3;
}  ;
#line 16
struct __nesc_attr_dmemcpy {
#line 16
  int a1, a2, a3;
}  ;
#line 17
struct __nesc_attr_nts {
}  ;
# 38 "/usr/msp430/include/sys/inttypes.h" 3
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
# 385 "/usr/lib/ncc/nesc_nx.h"
typedef struct { unsigned char data[1]; } __attribute__((packed)) nx_int8_t;typedef int8_t __nesc_nxbase_nx_int8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nx_int16_t;typedef int16_t __nesc_nxbase_nx_int16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nx_int32_t;typedef int32_t __nesc_nxbase_nx_int32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nx_int64_t;typedef int64_t __nesc_nxbase_nx_int64_t  ;
typedef struct { unsigned char data[1]; } __attribute__((packed)) nx_uint8_t;typedef uint8_t __nesc_nxbase_nx_uint8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nx_uint16_t;typedef uint16_t __nesc_nxbase_nx_uint16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nx_uint32_t;typedef uint32_t __nesc_nxbase_nx_uint32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nx_uint64_t;typedef uint64_t __nesc_nxbase_nx_uint64_t  ;


typedef struct { unsigned char data[1]; } __attribute__((packed)) nxle_int8_t;typedef int8_t __nesc_nxbase_nxle_int8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nxle_int16_t;typedef int16_t __nesc_nxbase_nxle_int16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nxle_int32_t;typedef int32_t __nesc_nxbase_nxle_int32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nxle_int64_t;typedef int64_t __nesc_nxbase_nxle_int64_t  ;
typedef struct { unsigned char data[1]; } __attribute__((packed)) nxle_uint8_t;typedef uint8_t __nesc_nxbase_nxle_uint8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nxle_uint16_t;typedef uint16_t __nesc_nxbase_nxle_uint16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nxle_uint32_t;typedef uint32_t __nesc_nxbase_nxle_uint32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nxle_uint64_t;typedef uint64_t __nesc_nxbase_nxle_uint64_t  ;
# 41 "/usr/msp430/include/sys/types.h" 3
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
# 42 "/usr/msp430/include/string.h" 3
extern void *memset(void *arg_0x40280220, int arg_0x40280378, size_t arg_0x40280510);
#line 63
extern void *memset(void *arg_0x4028c118, int arg_0x4028c270, size_t arg_0x4028c408);
# 59 "/usr/msp430/include/stdlib.h" 3
#line 56
typedef struct __nesc_unnamed4242 {
  int quot;
  int rem;
} div_t;







#line 64
typedef struct __nesc_unnamed4243 {
  long quot;
  long rem;
} ldiv_t;
# 122 "/usr/msp430/include/sys/config.h" 3
typedef long int __int32_t;
typedef unsigned long int __uint32_t;
# 12 "/usr/msp430/include/sys/_types.h" 3
typedef long _off_t;
typedef long _ssize_t;
# 28 "/usr/msp430/include/sys/reent.h" 3
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

  void (*__cleanup)(struct _reent *arg_0x402ae510);


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


  void (**_sig_func)(int arg_0x402b2b88);




  struct _glue __sglue;
  struct __sFILE __sf[3];
};
#line 273
struct _reent;
# 18 "/usr/msp430/include/math.h" 3
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
# 23 "/opt/tinyos-2.1.0/tos/system/tos.h"
typedef uint8_t bool;
enum __nesc_unnamed4247 {
#line 24
  FALSE = 0, TRUE = 1
};
typedef nx_int8_t nx_bool;







struct __nesc_attr_atmostonce {
};
#line 35
struct __nesc_attr_atleastonce {
};
#line 36
struct __nesc_attr_exactlyonce {
};
# 40 "/opt/tinyos-2.1.0/tos/types/TinyError.h"
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
  EALREADY = 9, 
  ENOMEM = 10, 
  ENOACK = 11, 
  ELAST = 11
};

typedef uint8_t error_t  ;

static inline error_t ecombine(error_t r1, error_t r2)  ;
# 39 "/usr/msp430/include/msp430/iostructures.h" 3
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
#line 108
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
# 116 "/usr/msp430/include/msp430/gpio.h" 3
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
# 94 "/usr/msp430/include/msp430/usart.h" 3
volatile unsigned char U0TCTL __asm ("0x0071");
#line 277
volatile unsigned char U1TCTL __asm ("0x0079");
# 27 "/usr/msp430/include/msp430/timera.h" 3
volatile unsigned int TA0CTL __asm ("0x0160");

volatile unsigned int TA0R __asm ("0x0170");


volatile unsigned int TA0CCTL0 __asm ("0x0162");

volatile unsigned int TA0CCTL1 __asm ("0x0164");
#line 70
volatile unsigned int TA0CCTL2 __asm ("0x0166");
#line 127
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
# 26 "/usr/msp430/include/msp430/timerb.h" 3
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
# 20 "/usr/msp430/include/msp430/basic_clock.h" 3
volatile unsigned char DCOCTL __asm ("0x0056");

volatile unsigned char BCSCTL1 __asm ("0x0057");

volatile unsigned char BCSCTL2 __asm ("0x0058");
# 18 "/usr/msp430/include/msp430/adc12.h" 3
volatile unsigned int ADC12CTL0 __asm ("0x01A0");

volatile unsigned int ADC12CTL1 __asm ("0x01A2");
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
# 83 "/usr/msp430/include/msp430x16x.h" 3
volatile unsigned char ME1 __asm ("0x0004");





volatile unsigned char ME2 __asm ("0x0005");
# 158 "/opt/tinyos-2.1.0/tos/chips/msp430/msp430hardware.h"
static volatile uint8_t U0CTLnr __asm ("0x0070");
static volatile uint8_t I2CTCTLnr __asm ("0x0071");
static volatile uint8_t I2CDCTLnr __asm ("0x0072");
#line 193
typedef uint8_t mcu_power_t  ;
static inline mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)  ;


enum __nesc_unnamed4257 {
  MSP430_POWER_ACTIVE = 0, 
  MSP430_POWER_LPM0 = 1, 
  MSP430_POWER_LPM1 = 2, 
  MSP430_POWER_LPM2 = 3, 
  MSP430_POWER_LPM3 = 4, 
  MSP430_POWER_LPM4 = 5
};

static inline void __nesc_disable_interrupt(void )  ;





static inline void __nesc_enable_interrupt(void )  ;




typedef bool __nesc_atomic_t;
__nesc_atomic_t __nesc_atomic_start(void );
void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts);






__nesc_atomic_t __nesc_atomic_start(void )   ;







void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)   ;
# 33 "/opt/tinyos-2.1.0/tos/platforms/telosb/hardware.h"
static inline void TOSH_SET_SIMO0_PIN()  ;
#line 33
static inline void TOSH_CLR_SIMO0_PIN()  ;
#line 33
static inline void TOSH_MAKE_SIMO0_OUTPUT()  ;
static inline void TOSH_SET_UCLK0_PIN()  ;
#line 34
static inline void TOSH_CLR_UCLK0_PIN()  ;
#line 34
static inline void TOSH_MAKE_UCLK0_OUTPUT()  ;
#line 76
enum __nesc_unnamed4258 {

  TOSH_HUMIDITY_ADDR = 5, 
  TOSH_HUMIDTEMP_ADDR = 3, 
  TOSH_HUMIDITY_RESET = 0x1E
};



static inline void TOSH_SET_FLASH_CS_PIN()  ;
#line 85
static inline void TOSH_CLR_FLASH_CS_PIN()  ;
#line 85
static inline void TOSH_MAKE_FLASH_CS_OUTPUT()  ;
static inline void TOSH_SET_FLASH_HOLD_PIN()  ;
#line 86
static inline void TOSH_MAKE_FLASH_HOLD_OUTPUT()  ;
# 32 "/opt/tinyos-2.1.0/tos/types/Leds.h"
enum __nesc_unnamed4259 {
  LEDS_LED0 = 1 << 0, 
  LEDS_LED1 = 1 << 1, 
  LEDS_LED2 = 1 << 2, 
  LEDS_LED3 = 1 << 3, 
  LEDS_LED4 = 1 << 4, 
  LEDS_LED5 = 1 << 5, 
  LEDS_LED6 = 1 << 6, 
  LEDS_LED7 = 1 << 7
};
# 29 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.h"
typedef struct __nesc_unnamed4260 {
#line 29
  int notUsed;
} 
#line 29
TMilli;
typedef struct __nesc_unnamed4261 {
#line 30
  int notUsed;
} 
#line 30
T32khz;
typedef struct __nesc_unnamed4262 {
#line 31
  int notUsed;
} 
#line 31
TMicro;
# 28 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.h"
enum __nesc_unnamed4263 {
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
typedef struct __nesc_unnamed4264 {

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
typedef struct __nesc_unnamed4265 {

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
typedef struct __nesc_unnamed4266 {

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
# 59 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12.h"
#line 48
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








#line 61
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
# 33 "/opt/tinyos-2.1.0/tos/types/Resource.h"
typedef uint8_t resource_client_id_t;
typedef uint16_t DarkC$Light$val_t;
typedef TMilli DarkC$TheftTimer$precision_tag;
typedef uint16_t AdcP$Read$val_t;
typedef uint16_t AdcP$ReadNow$val_t;
typedef const msp430adc12_channel_config_t *AdcP$Config$adc_config_t;
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
typedef TMilli /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$LocalTime$precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$precision_tag;
typedef uint32_t /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$size_type;
enum /*AntiTheftAppC.ParLight.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$__nesc_unnamed4270 {
  Msp430Adc12ClientAutoRVGC$0$ID = 0U
};
typedef const msp430adc12_channel_config_t */*AntiTheftAppC.ParLight.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$0$ConfSub$adc_config_t;
typedef const msp430adc12_channel_config_t */*AntiTheftAppC.ParLight.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$0$ConfUp$adc_config_t;
enum /*AntiTheftAppC.ParLight.AdcReadClientC*/AdcReadClientC$0$__nesc_unnamed4271 {
  AdcReadClientC$0$CLIENT = 0U
};
typedef TMilli AdcStreamP$Alarm$precision_tag;
typedef uint32_t AdcStreamP$Alarm$size_type;
typedef const msp430adc12_channel_config_t *AdcStreamP$AdcConfigure$adc_config_t;
typedef uint16_t AdcStreamP$ReadStream$val_t;
enum /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Timer*/Msp430Timer32khzC$1$__nesc_unnamed4272 {
  Msp430Timer32khzC$1$ALARM_ID = 1U
};
typedef T32khz /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$frequency_tag;
typedef /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$frequency_tag /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Alarm$precision_tag;
typedef uint16_t /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Alarm$size_type;
typedef TMilli /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$to_precision_tag;
typedef uint32_t /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$to_size_type;
typedef T32khz /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$from_precision_tag;
typedef uint16_t /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$from_size_type;
typedef /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$to_precision_tag /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$Alarm$precision_tag;
typedef /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$to_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$Alarm$size_type;
typedef /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$from_precision_tag /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$AlarmFrom$precision_tag;
typedef /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$from_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$AlarmFrom$size_type;
typedef /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$to_precision_tag /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$Counter$precision_tag;
typedef /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$to_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$Counter$size_type;
typedef uint16_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$val_t;
typedef /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$val_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$Service$val_t;
typedef /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$val_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$ReadStream$val_t;
enum /*AntiTheftAppC.ParLight.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$1$__nesc_unnamed4273 {
  Msp430Adc12ClientAutoRVGC$1$ID = 1U
};
typedef const msp430adc12_channel_config_t */*AntiTheftAppC.ParLight.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$1$ConfSub$adc_config_t;
typedef const msp430adc12_channel_config_t */*AntiTheftAppC.ParLight.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$1$ConfUp$adc_config_t;
enum /*AntiTheftAppC.ParLight.AdcReadStreamClientC*/AdcReadStreamClientC$0$__nesc_unnamed4274 {
  AdcReadStreamClientC$0$RSCLIENT = 0U
};
typedef const msp430adc12_channel_config_t *HamamatsuS1087ParP$AdcConfigure$adc_config_t;
# 49 "/opt/tinyos-2.1.0/tos/interfaces/Boot.nc"
static void DarkC$Boot$booted(void );
# 63 "/opt/tinyos-2.1.0/tos/interfaces/Read.nc"
static void DarkC$Light$readDone(error_t result, DarkC$Light$val_t val);
# 72 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
static void DarkC$TheftTimer$fired(void );
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t PlatformP$Init$init(void );
#line 51
static error_t MotePlatformC$Init$init(void );
# 35 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP$Msp430ClockInit$defaultInitClocks(void );
#line 32
static void Msp430ClockP$Msp430ClockInit$default$initTimerB(void );



static void Msp430ClockP$Msp430ClockInit$defaultInitTimerA(void );
#line 31
static void Msp430ClockP$Msp430ClockInit$default$initTimerA(void );





static void Msp430ClockP$Msp430ClockInit$defaultInitTimerB(void );
#line 34
static void Msp430ClockP$Msp430ClockInit$defaultSetupDcoCalibrate(void );
#line 29
static void Msp430ClockP$Msp430ClockInit$default$setupDcoCalibrate(void );
static void Msp430ClockP$Msp430ClockInit$default$initClocks(void );
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t Msp430ClockP$Init$init(void );
# 28 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX0$fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Overflow$fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX1$fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$default$fired(
# 40 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x40615428);
# 41 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$clear(void );


static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setClockSource(uint16_t clockSource);
#line 43
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$disableEvents(void );
#line 39
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setMode(int mode);





static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setInputDivider(uint16_t inputDivider);
# 28 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX0$fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Overflow$fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX1$fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$default$fired(
# 40 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x40615428);
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$get(void );
static bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$isOverflowPending(void );
# 33 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$default$captured(uint16_t time);
# 31 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Control$getControl(void );



static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Control$setControl(msp430_compare_control_t control);
# 28 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Event$fired(void );
# 30 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$setEvent(uint16_t time);
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Timer$overflow(void );
# 33 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$default$captured(uint16_t time);
# 31 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Control$getControl(void );



static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Control$setControl(msp430_compare_control_t control);
# 28 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Event$fired(void );
# 30 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$setEvent(uint16_t time);
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Timer$overflow(void );
# 33 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$default$captured(uint16_t time);
# 31 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Control$getControl(void );
# 28 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Event$fired(void );
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$default$fired(void );
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Timer$overflow(void );
# 33 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$default$captured(uint16_t time);
# 31 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$getControl(void );
#line 46
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$enableEvents(void );
#line 36
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$setControlAsCompare(void );










static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$disableEvents(void );
#line 33
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$clearPendingInterrupt(void );
# 28 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Event$fired(void );
# 30 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEventFromNow(uint16_t delta);
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$overflow(void );
# 33 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$default$captured(uint16_t time);
# 31 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$getControl(void );
# 28 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Event$fired(void );
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$default$fired(void );
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Timer$overflow(void );
# 33 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$default$captured(uint16_t time);
# 31 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$getControl(void );
#line 46
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$enableEvents(void );
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$disableEvents(void );
#line 33
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$clearPendingInterrupt(void );
# 28 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Event$fired(void );
# 30 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$setEventFromNow(uint16_t delta);
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Timer$overflow(void );
# 33 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$default$captured(uint16_t time);
# 31 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Control$getControl(void );
# 28 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Event$fired(void );
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$default$fired(void );
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Timer$overflow(void );
# 33 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$default$captured(uint16_t time);
# 31 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Control$getControl(void );
# 28 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Event$fired(void );
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$default$fired(void );
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Timer$overflow(void );
# 33 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$default$captured(uint16_t time);
# 31 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Control$getControl(void );
# 28 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Event$fired(void );
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$default$fired(void );
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Timer$overflow(void );
# 33 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$default$captured(uint16_t time);
# 31 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Control$getControl(void );
# 28 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Event$fired(void );
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$default$fired(void );
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Timer$overflow(void );
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t SchedulerBasicP$TaskBasic$postTask(
# 45 "/opt/tinyos-2.1.0/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x405744a0);
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP$TaskBasic$default$runTask(
# 45 "/opt/tinyos-2.1.0/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x405744a0);
# 46 "/opt/tinyos-2.1.0/tos/interfaces/Scheduler.nc"
static void SchedulerBasicP$Scheduler$init(void );
#line 61
static void SchedulerBasicP$Scheduler$taskLoop(void );
#line 54
static bool SchedulerBasicP$Scheduler$runNextTask(void );
# 54 "/opt/tinyos-2.1.0/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t McuSleepC$McuPowerOverride$default$lowestState(void );
# 59 "/opt/tinyos-2.1.0/tos/interfaces/McuSleep.nc"
static void McuSleepC$McuSleep$sleep(void );
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t LedsP$Init$init(void );
# 83 "/opt/tinyos-2.1.0/tos/interfaces/Leds.nc"
static void LedsP$Leds$led2Off(void );
#line 78
static void LedsP$Leds$led2On(void );
# 71 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$set(void );
#line 71
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$set(void );
#line 71
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$set(void );




static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$clr(void );
#line 64
static void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$makeInput(void );
#line 85
static void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$selectModuleFunc(void );
#line 64
static void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$makeInput(void );
#line 85
static void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$selectModuleFunc(void );
#line 64
static void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$makeInput(void );
#line 85
static void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$selectModuleFunc(void );
#line 64
static void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$makeInput(void );
#line 85
static void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$selectModuleFunc(void );
#line 64
static void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$makeInput(void );
#line 85
static void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$selectModuleFunc(void );
#line 64
static void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$makeInput(void );
#line 85
static void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$selectModuleFunc(void );
#line 64
static void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$makeInput(void );
#line 85
static void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$selectModuleFunc(void );
#line 64
static void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$makeInput(void );
#line 85
static void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$selectModuleFunc(void );
# 35 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$makeOutput(void );
#line 29
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$set(void );





static void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$makeOutput(void );
#line 29
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$set(void );





static void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$makeOutput(void );
#line 29
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$set(void );
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$clr(void );
# 55 "/opt/tinyos-2.1.0/tos/interfaces/Read.nc"
static error_t AdcP$Read$read(
# 38 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40801668);
# 63 "/opt/tinyos-2.1.0/tos/interfaces/Read.nc"
static void AdcP$Read$default$readDone(
# 38 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40801668, 
# 63 "/opt/tinyos-2.1.0/tos/interfaces/Read.nc"
error_t result, AdcP$Read$val_t val);
# 66 "/opt/tinyos-2.1.0/tos/interfaces/ReadNow.nc"
static void AdcP$ReadNow$default$readDone(
# 39 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40818d70, 
# 66 "/opt/tinyos-2.1.0/tos/interfaces/ReadNow.nc"
error_t result, AdcP$ReadNow$val_t val);
# 58 "/opt/tinyos-2.1.0/tos/interfaces/AdcConfigure.nc"
static AdcP$Config$adc_config_t AdcP$Config$default$getConfiguration(
# 48 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40812068);
# 189 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t AdcP$SingleChannel$default$getData(
# 49 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40823e40);
# 84 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t AdcP$SingleChannel$default$configureSingle(
# 49 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40823e40, 
# 84 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t * config);
#line 227
static uint16_t * AdcP$SingleChannel$multipleDataReady(
# 49 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40823e40, 
# 227 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t * buffer, uint16_t numSamples);
#line 206
static error_t AdcP$SingleChannel$singleDataReady(
# 49 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40823e40, 
# 206 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t data);
# 110 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t AdcP$ResourceRead$default$release(
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40817890);
# 78 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t AdcP$ResourceRead$default$request(
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40817890);
# 92 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static void AdcP$ResourceRead$granted(
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40817890);
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void AdcP$readDone$runTask(void );
# 105 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc"
static void Msp430Adc12ImplP$MultiChannel$default$dataReady(
# 42 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x40858108, 
# 105 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc"
uint16_t *buffer, uint16_t numSamples);
# 112 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/HplAdc12.nc"
static void Msp430Adc12ImplP$HplAdc12$conversionDone(uint16_t iv);
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
static void Msp430Adc12ImplP$CompareA1$fired(void );
# 49 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
static void Msp430Adc12ImplP$Overflow$default$memOverflow(
# 43 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x408589f8);
# 54 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
static void Msp430Adc12ImplP$Overflow$default$conversionTimeOverflow(
# 43 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x408589f8);
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t Msp430Adc12ImplP$Init$init(void );
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
static void Msp430Adc12ImplP$TimerA$overflow(void );
# 189 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t Msp430Adc12ImplP$SingleChannel$getData(
# 41 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x408593f0);
# 84 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t Msp430Adc12ImplP$SingleChannel$configureSingle(
# 41 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x408593f0, 
# 84 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t * config);
#line 227
static uint16_t * Msp430Adc12ImplP$SingleChannel$default$multipleDataReady(
# 41 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x408593f0, 
# 227 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t * buffer, uint16_t numSamples);
#line 138
static error_t Msp430Adc12ImplP$SingleChannel$configureMultiple(
# 41 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x408593f0, 
# 138 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t * config, uint16_t * buffer, uint16_t numSamples, uint16_t jiffies);
#line 206
static error_t Msp430Adc12ImplP$SingleChannel$default$singleDataReady(
# 41 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x408593f0, 
# 206 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t data);
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
static void Msp430Adc12ImplP$CompareA0$fired(void );
# 63 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/HplAdc12.nc"
static adc12ctl0_t HplAdc12P$HplAdc12$getCtl0(void );
#line 82
static adc12memctl_t HplAdc12P$HplAdc12$getMCtl(uint8_t idx);
#line 106
static void HplAdc12P$HplAdc12$resetIFGs(void );
#line 118
static bool HplAdc12P$HplAdc12$isBusy(void );
#line 75
static void HplAdc12P$HplAdc12$setMCtl(uint8_t idx, adc12memctl_t memControl);
#line 128
static void HplAdc12P$HplAdc12$startConversion(void );
#line 51
static void HplAdc12P$HplAdc12$setCtl0(adc12ctl0_t control0);
#line 89
static uint16_t HplAdc12P$HplAdc12$getMem(uint8_t idx);





static void HplAdc12P$HplAdc12$setIEFlags(uint16_t mask);
#line 123
static void HplAdc12P$HplAdc12$stopConversion(void );









static void HplAdc12P$HplAdc12$enableConversion(void );
#line 57
static void HplAdc12P$HplAdc12$setCtl1(adc12ctl1_t control1);
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$Init$init(void );
# 69 "/opt/tinyos-2.1.0/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$enqueue(resource_client_id_t id);
#line 43
static bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$isEmpty(void );








static bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$isEnqueued(resource_client_id_t id);







static resource_client_id_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$dequeue(void );
# 43 "/opt/tinyos-2.1.0/tos/interfaces/ResourceRequested.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceRequested$default$requested(
# 52 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x40903318);
# 55 "/opt/tinyos-2.1.0/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$unconfigure(
# 56 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x40902010);
# 49 "/opt/tinyos-2.1.0/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$configure(
# 56 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x40902010);
# 110 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$release(
# 51 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x409048e0);
# 78 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$request(
# 51 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x409048e0);
# 92 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$default$granted(
# 51 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x409048e0);
# 88 "/opt/tinyos-2.1.0/tos/interfaces/ArbiterInfo.nc"
static uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ArbiterInfo$userId(void );
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask$runTask(void );
# 112 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/HplAdc12.nc"
static void Msp430RefVoltGeneratorP$HplAdc12$conversionDone(uint16_t iv);
# 72 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
static void Msp430RefVoltGeneratorP$SwitchOffTimer$fired(void );
# 83 "/opt/tinyos-2.1.0/tos/interfaces/SplitControl.nc"
static error_t Msp430RefVoltGeneratorP$RefVolt_2_5V$start(void );
#line 83
static error_t Msp430RefVoltGeneratorP$RefVolt_1_5V$start(void );
#line 109
static error_t Msp430RefVoltGeneratorP$RefVolt_1_5V$stop(void );
# 72 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
static void Msp430RefVoltGeneratorP$SwitchOnTimer$fired(void );
# 58 "/opt/tinyos-2.1.0/tos/interfaces/AdcConfigure.nc"
static Msp430RefVoltArbiterImplP$Config$adc_config_t Msp430RefVoltArbiterImplP$Config$default$getConfiguration(
# 43 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x40968d10);
# 92 "/opt/tinyos-2.1.0/tos/interfaces/SplitControl.nc"
static void Msp430RefVoltArbiterImplP$RefVolt_2_5V$startDone(error_t error);
#line 117
static void Msp430RefVoltArbiterImplP$RefVolt_2_5V$stopDone(error_t error);
# 110 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t Msp430RefVoltArbiterImplP$AdcResource$default$release(
# 40 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x409692b8);
# 78 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t Msp430RefVoltArbiterImplP$AdcResource$default$request(
# 40 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x409692b8);
# 92 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static void Msp430RefVoltArbiterImplP$AdcResource$granted(
# 40 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x409692b8);
# 110 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t Msp430RefVoltArbiterImplP$ClientResource$release(
# 38 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x409357d0);
# 78 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t Msp430RefVoltArbiterImplP$ClientResource$request(
# 38 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x409357d0);
# 92 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static void Msp430RefVoltArbiterImplP$ClientResource$default$granted(
# 38 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x409357d0);
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void Msp430RefVoltArbiterImplP$switchOff$runTask(void );
# 92 "/opt/tinyos-2.1.0/tos/interfaces/SplitControl.nc"
static void Msp430RefVoltArbiterImplP$RefVolt_1_5V$startDone(error_t error);
#line 117
static void Msp430RefVoltArbiterImplP$RefVolt_1_5V$stopDone(error_t error);
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$fired(void );
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$overflow(void );
# 92 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$startAt(/*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$size_type t0, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$size_type dt);
#line 62
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$stop(void );
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Init$init(void );
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$overflow(void );
# 53 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
static /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$size_type /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$get(void );






static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$overflow(void );
#line 53
static /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$size_type /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$get(void );
# 98 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getNow(void );
#line 92
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$size_type dt);
#line 105
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getAlarm(void );
#line 62
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$fired(void );
# 71 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$overflow(void );
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$runTask(void );
# 67 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$fired(void );
# 125 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$getNow(void );
#line 118
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$startOneShotAt(uint32_t t0, uint32_t dt);
#line 67
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$stop(void );
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$runTask(void );
# 72 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$fired(void );
#line 72
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$default$fired(
# 37 "/opt/tinyos-2.1.0/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x40a2b9f0);
# 53 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startPeriodic(
# 37 "/opt/tinyos-2.1.0/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x40a2b9f0, 
# 53 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
uint32_t dt);








static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShot(
# 37 "/opt/tinyos-2.1.0/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x40a2b9f0, 
# 62 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
uint32_t dt);




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$stop(
# 37 "/opt/tinyos-2.1.0/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x40a2b9f0);
# 71 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$overflow(void );
# 58 "/opt/tinyos-2.1.0/tos/interfaces/AdcConfigure.nc"
static /*AntiTheftAppC.ParLight.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$0$ConfSub$adc_config_t /*AntiTheftAppC.ParLight.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$0$ConfSub$getConfiguration(void );
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void AdcStreamP$bufferDone$runTask(void );
#line 64
static void AdcStreamP$readStreamDone$runTask(void );
#line 64
static void AdcStreamP$readStreamFail$runTask(void );
# 67 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
static void AdcStreamP$Alarm$fired(void );
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t AdcStreamP$Init$init(void );
# 58 "/opt/tinyos-2.1.0/tos/interfaces/AdcConfigure.nc"
static AdcStreamP$AdcConfigure$adc_config_t AdcStreamP$AdcConfigure$default$getConfiguration(
# 53 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x40aa7428);
# 189 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t AdcStreamP$SingleChannel$default$getData(
# 52 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x40aa8598);
# 84 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t AdcStreamP$SingleChannel$default$configureSingle(
# 52 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x40aa8598, 
# 84 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t * config);
#line 227
static uint16_t * AdcStreamP$SingleChannel$multipleDataReady(
# 52 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x40aa8598, 
# 227 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t * buffer, uint16_t numSamples);
#line 138
static error_t AdcStreamP$SingleChannel$default$configureMultiple(
# 52 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x40aa8598, 
# 138 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t * config, uint16_t * buffer, uint16_t numSamples, uint16_t jiffies);
#line 206
static error_t AdcStreamP$SingleChannel$singleDataReady(
# 52 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x40aa8598, 
# 206 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t data);
# 78 "/opt/tinyos-2.1.0/tos/interfaces/ReadStream.nc"
static error_t AdcStreamP$ReadStream$read(
# 49 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x40a869f0, 
# 78 "/opt/tinyos-2.1.0/tos/interfaces/ReadStream.nc"
uint32_t usPeriod);
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Msp430Compare$fired(void );
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Msp430Timer$overflow(void );
# 92 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Alarm$startAt(/*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Alarm$size_type t0, /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Alarm$size_type dt);





static /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$Alarm$size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$Alarm$getNow(void );
#line 92
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$Alarm$startAt(/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$Alarm$size_type t0, /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$Alarm$size_type dt);
#line 67
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$AlarmFrom$fired(void );
# 71 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$Counter$overflow(void );
# 89 "/opt/tinyos-2.1.0/tos/interfaces/ReadStream.nc"
static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$Service$bufferDone(
# 26 "/opt/tinyos-2.1.0/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x40adad00, 
# 89 "/opt/tinyos-2.1.0/tos/interfaces/ReadStream.nc"
error_t result, 
#line 86
/*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$Service$val_t * buf, 



uint16_t count);
#line 102
static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$Service$readDone(
# 26 "/opt/tinyos-2.1.0/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x40adad00, 
# 102 "/opt/tinyos-2.1.0/tos/interfaces/ReadStream.nc"
error_t result, uint32_t usActualPeriod);
#line 89
static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$ReadStream$default$bufferDone(
# 24 "/opt/tinyos-2.1.0/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x40adb258, 
# 89 "/opt/tinyos-2.1.0/tos/interfaces/ReadStream.nc"
error_t result, 
#line 86
/*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$ReadStream$val_t * buf, 



uint16_t count);
#line 102
static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$ReadStream$default$readDone(
# 24 "/opt/tinyos-2.1.0/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x40adb258, 
# 102 "/opt/tinyos-2.1.0/tos/interfaces/ReadStream.nc"
error_t result, uint32_t usActualPeriod);
# 110 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$Resource$default$release(
# 27 "/opt/tinyos-2.1.0/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x40ad7678);
# 92 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$Resource$granted(
# 27 "/opt/tinyos-2.1.0/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x40ad7678);
# 58 "/opt/tinyos-2.1.0/tos/interfaces/AdcConfigure.nc"
static /*AntiTheftAppC.ParLight.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$1$ConfSub$adc_config_t /*AntiTheftAppC.ParLight.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$1$ConfSub$getConfiguration(void );
#line 58
static HamamatsuS1087ParP$AdcConfigure$adc_config_t HamamatsuS1087ParP$AdcConfigure$getConfiguration(void );
# 55 "/opt/tinyos-2.1.0/tos/interfaces/Read.nc"
static error_t DarkC$Light$read(void );
# 83 "/opt/tinyos-2.1.0/tos/interfaces/Leds.nc"
static void DarkC$Leds$led2Off(void );
#line 78
static void DarkC$Leds$led2On(void );
# 53 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
static void DarkC$TheftTimer$startPeriodic(uint32_t dt);
# 17 "DarkC.nc"
enum DarkC$__nesc_unnamed4275 {

  DarkC$DARK_INTERVAL = 256, 
  DarkC$DARK_THRESHOLD = 30
};

static inline void DarkC$Boot$booted(void );




static inline void DarkC$TheftTimer$fired(void );




static void DarkC$Light$readDone(error_t ok, uint16_t val);
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t PlatformP$MoteInit$init(void );
#line 51
static error_t PlatformP$MoteClockInit$init(void );
#line 51
static error_t PlatformP$LedsInit$init(void );
# 10 "/opt/tinyos-2.1.0/tos/platforms/telosa/PlatformP.nc"
static inline error_t PlatformP$Init$init(void );
# 6 "/opt/tinyos-2.1.0/tos/platforms/telosb/MotePlatformC.nc"
static __inline void MotePlatformC$uwait(uint16_t u);




static __inline void MotePlatformC$TOSH_wait(void );




static void MotePlatformC$TOSH_FLASH_M25P_DP_bit(bool set);










static inline void MotePlatformC$TOSH_FLASH_M25P_DP(void );
#line 56
static inline error_t MotePlatformC$Init$init(void );
# 32 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP$Msp430ClockInit$initTimerB(void );
#line 31
static void Msp430ClockP$Msp430ClockInit$initTimerA(void );
#line 29
static void Msp430ClockP$Msp430ClockInit$setupDcoCalibrate(void );
static void Msp430ClockP$Msp430ClockInit$initClocks(void );
# 39 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430ClockP.nc"
static volatile uint8_t Msp430ClockP$IE1 __asm ("0x0000");
static volatile uint16_t Msp430ClockP$TA0CTL __asm ("0x0160");
static volatile uint16_t Msp430ClockP$TA0IV __asm ("0x012E");
static volatile uint16_t Msp430ClockP$TBCTL __asm ("0x0180");
static volatile uint16_t Msp430ClockP$TBIV __asm ("0x011E");

enum Msp430ClockP$__nesc_unnamed4276 {

  Msp430ClockP$ACLK_CALIB_PERIOD = 8, 
  Msp430ClockP$TARGET_DCO_DELTA = 4096 / 32 * Msp430ClockP$ACLK_CALIB_PERIOD
};


static inline void Msp430ClockP$Msp430ClockInit$defaultSetupDcoCalibrate(void );
#line 64
static inline void Msp430ClockP$Msp430ClockInit$defaultInitClocks(void );
#line 85
static inline void Msp430ClockP$Msp430ClockInit$defaultInitTimerA(void );
#line 100
static inline void Msp430ClockP$Msp430ClockInit$defaultInitTimerB(void );
#line 115
static inline void Msp430ClockP$Msp430ClockInit$default$setupDcoCalibrate(void );




static inline void Msp430ClockP$Msp430ClockInit$default$initClocks(void );




static inline void Msp430ClockP$Msp430ClockInit$default$initTimerA(void );




static inline void Msp430ClockP$Msp430ClockInit$default$initTimerB(void );





static inline void Msp430ClockP$startTimerA(void );
#line 148
static inline void Msp430ClockP$startTimerB(void );
#line 160
static void Msp430ClockP$set_dco_calib(int calib);





static inline uint16_t Msp430ClockP$test_calib_busywait_delta(int calib);
#line 189
static inline void Msp430ClockP$busyCalibrateDco(void );
#line 214
static inline error_t Msp430ClockP$Init$init(void );
# 28 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$fired(
# 40 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x40615428);
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$overflow(void );
# 80 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerP.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setMode(int mode);









static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$clear(void );









static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$disableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setClockSource(uint16_t clockSource);




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setInputDivider(uint16_t inputDivider);




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX0$fired(void );




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX1$fired(void );





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Overflow$fired(void );








static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$default$fired(uint8_t n);
# 28 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$fired(
# 40 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x40615428);
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$overflow(void );
# 51 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$get(void );
#line 70
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$isOverflowPending(void );
#line 115
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX0$fired(void );




static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX1$fired(void );





static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Overflow$fired(void );








static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$default$fired(uint8_t n);
# 75 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$captured(uint16_t time);
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$fired(void );
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$CC2int(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Control$getControl(void );
#line 89
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Control$setControl(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t x);
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$setEvent(uint16_t x);
#line 169
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Event$fired(void );







static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$default$captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Timer$overflow(void );
# 75 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$captured(uint16_t time);
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$fired(void );
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$CC2int(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Control$getControl(void );
#line 89
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Control$setControl(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t x);
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$setEvent(uint16_t x);
#line 169
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Event$fired(void );







static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$default$captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Timer$overflow(void );
# 75 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$captured(uint16_t time);
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$fired(void );
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$cc_t;


static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Control$getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$getEvent(void );
#line 169
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Event$fired(void );







static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$default$captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$default$fired(void );



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Timer$overflow(void );
# 75 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$captured(uint16_t time);
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$fired(void );
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$get(void );
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$int2CC(uint16_t x)  ;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$compareControl(void );
#line 74
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$clearPendingInterrupt(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$setControlAsCompare(void );
#line 119
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$disableEvents(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEventFromNow(uint16_t x);
#line 169
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Event$fired(void );







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$default$captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$overflow(void );
# 75 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$captured(uint16_t time);
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$fired(void );
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$cc_t;


static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Event$fired(void );







static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$default$captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$default$fired(void );



static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Timer$overflow(void );
# 75 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$captured(uint16_t time);
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$fired(void );
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Timer$get(void );
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$cc_t;


static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$clearPendingInterrupt(void );
#line 119
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$disableEvents(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$setEventFromNow(uint16_t x);
#line 169
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Event$fired(void );







static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$default$captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Timer$overflow(void );
# 75 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$captured(uint16_t time);
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$fired(void );
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$cc_t;


static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Control$getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Event$fired(void );







static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$default$captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$default$fired(void );



static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Timer$overflow(void );
# 75 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$captured(uint16_t time);
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$fired(void );
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$cc_t;


static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Control$getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Event$fired(void );







static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$default$captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$default$fired(void );



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Timer$overflow(void );
# 75 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$captured(uint16_t time);
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$fired(void );
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$cc_t;


static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Control$getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Event$fired(void );







static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$default$captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$default$fired(void );



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Timer$overflow(void );
# 75 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$captured(uint16_t time);
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$fired(void );
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$cc_t;


static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Control$getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Event$fired(void );







static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$default$captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$default$fired(void );



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Timer$overflow(void );
# 28 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void Msp430TimerCommonP$VectorTimerB1$fired(void );
#line 28
static void Msp430TimerCommonP$VectorTimerA0$fired(void );
#line 28
static void Msp430TimerCommonP$VectorTimerA1$fired(void );
#line 28
static void Msp430TimerCommonP$VectorTimerB0$fired(void );
# 11 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
void sig_TIMERA0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(12)))  ;
void sig_TIMERA1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(10)))  ;
void sig_TIMERB0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(26)))  ;
void sig_TIMERB1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(24)))  ;
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t RealMainP$SoftwareInit$init(void );
# 49 "/opt/tinyos-2.1.0/tos/interfaces/Boot.nc"
static void RealMainP$Boot$booted(void );
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t RealMainP$PlatformInit$init(void );
# 46 "/opt/tinyos-2.1.0/tos/interfaces/Scheduler.nc"
static void RealMainP$Scheduler$init(void );
#line 61
static void RealMainP$Scheduler$taskLoop(void );
#line 54
static bool RealMainP$Scheduler$runNextTask(void );
# 52 "/opt/tinyos-2.1.0/tos/system/RealMainP.nc"
int main(void )   ;
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP$TaskBasic$runTask(
# 45 "/opt/tinyos-2.1.0/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x405744a0);
# 59 "/opt/tinyos-2.1.0/tos/interfaces/McuSleep.nc"
static void SchedulerBasicP$McuSleep$sleep(void );
# 50 "/opt/tinyos-2.1.0/tos/system/SchedulerBasicP.nc"
enum SchedulerBasicP$__nesc_unnamed4277 {

  SchedulerBasicP$NUM_TASKS = 8U, 
  SchedulerBasicP$NO_TASK = 255
};

uint8_t SchedulerBasicP$m_head;
uint8_t SchedulerBasicP$m_tail;
uint8_t SchedulerBasicP$m_next[SchedulerBasicP$NUM_TASKS];








static __inline uint8_t SchedulerBasicP$popTask(void );
#line 86
static inline bool SchedulerBasicP$isWaiting(uint8_t id);




static inline bool SchedulerBasicP$pushTask(uint8_t id);
#line 113
static inline void SchedulerBasicP$Scheduler$init(void );









static bool SchedulerBasicP$Scheduler$runNextTask(void );
#line 138
static inline void SchedulerBasicP$Scheduler$taskLoop(void );
#line 159
static error_t SchedulerBasicP$TaskBasic$postTask(uint8_t id);




static void SchedulerBasicP$TaskBasic$default$runTask(uint8_t id);
# 54 "/opt/tinyos-2.1.0/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t McuSleepC$McuPowerOverride$lowestState(void );
# 51 "/opt/tinyos-2.1.0/tos/chips/msp430/McuSleepC.nc"
bool McuSleepC$dirty = TRUE;
mcu_power_t McuSleepC$powerState = MSP430_POWER_ACTIVE;




const uint16_t McuSleepC$msp430PowerBits[MSP430_POWER_LPM4 + 1] = { 
0, 
0x0010, 
0x0040 + 0x0010, 
0x0080 + 0x0010, 
0x0080 + 0x0040 + 0x0010, 
0x0080 + 0x0040 + 0x0020 + 0x0010 };


static inline mcu_power_t McuSleepC$getPowerState(void );
#line 104
static inline void McuSleepC$computePowerState(void );




static inline void McuSleepC$McuSleep$sleep(void );
#line 126
static inline mcu_power_t McuSleepC$McuPowerOverride$default$lowestState(void );
# 35 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
static void LedsP$Led0$makeOutput(void );
#line 29
static void LedsP$Led0$set(void );





static void LedsP$Led1$makeOutput(void );
#line 29
static void LedsP$Led1$set(void );





static void LedsP$Led2$makeOutput(void );
#line 29
static void LedsP$Led2$set(void );
static void LedsP$Led2$clr(void );
# 45 "/opt/tinyos-2.1.0/tos/system/LedsP.nc"
static inline error_t LedsP$Init$init(void );
#line 93
static inline void LedsP$Leds$led2On(void );




static inline void LedsP$Leds$led2Off(void );
# 45 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$set(void );






static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$makeOutput(void );
#line 45
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$set(void );






static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$makeOutput(void );
#line 45
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$set(void );
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$clr(void );





static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$makeOutput(void );
#line 50
static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$makeInput(void );



static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$selectIOFunc(void );
#line 50
static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$makeInput(void );



static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$selectIOFunc(void );
#line 50
static inline void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$makeInput(void );



static inline void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$selectIOFunc(void );
#line 50
static inline void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$makeInput(void );



static inline void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$selectIOFunc(void );
#line 50
static inline void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$makeInput(void );



static inline void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$selectIOFunc(void );
#line 50
static inline void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$makeInput(void );



static inline void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$selectIOFunc(void );
#line 50
static inline void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$makeInput(void );



static inline void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$selectIOFunc(void );
#line 50
static inline void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$makeInput(void );



static inline void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$selectIOFunc(void );
# 71 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$makeOutput(void );
#line 34
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$set(void );
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$set(void );





static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$makeOutput(void );
# 71 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$makeOutput(void );
#line 34
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$set(void );
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$set(void );





static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$makeOutput(void );
# 71 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$makeOutput(void );
#line 34
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$set(void );




static void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$clr(void );
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$set(void );
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$clr(void );




static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$makeOutput(void );
# 63 "/opt/tinyos-2.1.0/tos/interfaces/Read.nc"
static void AdcP$Read$readDone(
# 38 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40801668, 
# 63 "/opt/tinyos-2.1.0/tos/interfaces/Read.nc"
error_t result, AdcP$Read$val_t val);
# 66 "/opt/tinyos-2.1.0/tos/interfaces/ReadNow.nc"
static void AdcP$ReadNow$readDone(
# 39 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40818d70, 
# 66 "/opt/tinyos-2.1.0/tos/interfaces/ReadNow.nc"
error_t result, AdcP$ReadNow$val_t val);
# 58 "/opt/tinyos-2.1.0/tos/interfaces/AdcConfigure.nc"
static AdcP$Config$adc_config_t AdcP$Config$getConfiguration(
# 48 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40812068);
# 189 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t AdcP$SingleChannel$getData(
# 49 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40823e40);
# 84 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t AdcP$SingleChannel$configureSingle(
# 49 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40823e40, 
# 84 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t * config);
# 110 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t AdcP$ResourceRead$release(
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40817890);
# 78 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t AdcP$ResourceRead$request(
# 44 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40817890);
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t AdcP$readDone$postTask(void );
# 136 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcP.nc"
enum AdcP$__nesc_unnamed4278 {
#line 136
  AdcP$readDone = 0U
};
#line 136
typedef int AdcP$__nesc_sillytask_readDone[AdcP$readDone];
#line 54
enum AdcP$__nesc_unnamed4279 {
  AdcP$STATE_READ, 
  AdcP$STATE_READNOW, 
  AdcP$STATE_READNOW_INVALID_CONFIG
};


uint8_t AdcP$state;
uint8_t AdcP$owner;
uint16_t AdcP$value;

static inline error_t AdcP$configure(uint8_t client);









static inline error_t AdcP$Read$read(uint8_t client);




static void AdcP$ResourceRead$granted(uint8_t client);
#line 136
static inline void AdcP$readDone$runTask(void );





static error_t AdcP$SingleChannel$singleDataReady(uint8_t client, uint16_t data);
#line 161
static inline uint16_t *AdcP$SingleChannel$multipleDataReady(uint8_t client, 
uint16_t *buf, uint16_t numSamples);





static inline error_t AdcP$ResourceRead$default$request(uint8_t client);

static inline error_t AdcP$ResourceRead$default$release(uint8_t client);

static inline void AdcP$Read$default$readDone(uint8_t client, error_t result, uint16_t val);





static inline void AdcP$ReadNow$default$readDone(uint8_t client, error_t result, uint16_t val);

static inline error_t AdcP$SingleChannel$default$getData(uint8_t client);




const msp430adc12_channel_config_t AdcP$defaultConfig = { INPUT_CHANNEL_NONE, 0, 0, 0, 0, 0, 0, 0 };
static inline const msp430adc12_channel_config_t *
AdcP$Config$default$getConfiguration(uint8_t client);



static inline error_t AdcP$SingleChannel$default$configureSingle(uint8_t client, 
const msp430adc12_channel_config_t *config);
# 105 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc"
static void Msp430Adc12ImplP$MultiChannel$dataReady(
# 42 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x40858108, 
# 105 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc"
uint16_t *buffer, uint16_t numSamples);
# 63 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/HplAdc12.nc"
static adc12ctl0_t Msp430Adc12ImplP$HplAdc12$getCtl0(void );
#line 82
static adc12memctl_t Msp430Adc12ImplP$HplAdc12$getMCtl(uint8_t idx);
#line 106
static void Msp430Adc12ImplP$HplAdc12$resetIFGs(void );
#line 75
static void Msp430Adc12ImplP$HplAdc12$setMCtl(uint8_t idx, adc12memctl_t memControl);
#line 128
static void Msp430Adc12ImplP$HplAdc12$startConversion(void );
#line 51
static void Msp430Adc12ImplP$HplAdc12$setCtl0(adc12ctl0_t control0);
#line 89
static uint16_t Msp430Adc12ImplP$HplAdc12$getMem(uint8_t idx);





static void Msp430Adc12ImplP$HplAdc12$setIEFlags(uint16_t mask);
#line 123
static void Msp430Adc12ImplP$HplAdc12$stopConversion(void );









static void Msp430Adc12ImplP$HplAdc12$enableConversion(void );
#line 57
static void Msp430Adc12ImplP$HplAdc12$setCtl1(adc12ctl1_t control1);
# 64 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void Msp430Adc12ImplP$Port64$makeInput(void );
#line 85
static void Msp430Adc12ImplP$Port64$selectIOFunc(void );
#line 78
static void Msp430Adc12ImplP$Port64$selectModuleFunc(void );
# 30 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
static void Msp430Adc12ImplP$CompareA1$setEvent(uint16_t time);
# 35 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void Msp430Adc12ImplP$ControlA0$setControl(msp430_compare_control_t control);
# 64 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void Msp430Adc12ImplP$Port62$makeInput(void );
#line 85
static void Msp430Adc12ImplP$Port62$selectIOFunc(void );
#line 78
static void Msp430Adc12ImplP$Port62$selectModuleFunc(void );
# 49 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
static void Msp430Adc12ImplP$Overflow$memOverflow(
# 43 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x408589f8);
# 54 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
static void Msp430Adc12ImplP$Overflow$conversionTimeOverflow(
# 43 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x408589f8);
# 64 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void Msp430Adc12ImplP$Port67$makeInput(void );
#line 85
static void Msp430Adc12ImplP$Port67$selectIOFunc(void );
#line 78
static void Msp430Adc12ImplP$Port67$selectModuleFunc(void );
#line 64
static void Msp430Adc12ImplP$Port60$makeInput(void );
#line 85
static void Msp430Adc12ImplP$Port60$selectIOFunc(void );
#line 78
static void Msp430Adc12ImplP$Port60$selectModuleFunc(void );
#line 64
static void Msp430Adc12ImplP$Port65$makeInput(void );
#line 85
static void Msp430Adc12ImplP$Port65$selectIOFunc(void );
#line 78
static void Msp430Adc12ImplP$Port65$selectModuleFunc(void );
# 41 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
static void Msp430Adc12ImplP$TimerA$clear(void );


static void Msp430Adc12ImplP$TimerA$setClockSource(uint16_t clockSource);
#line 43
static void Msp430Adc12ImplP$TimerA$disableEvents(void );
#line 39
static void Msp430Adc12ImplP$TimerA$setMode(int mode);





static void Msp430Adc12ImplP$TimerA$setInputDivider(uint16_t inputDivider);
# 88 "/opt/tinyos-2.1.0/tos/interfaces/ArbiterInfo.nc"
static uint8_t Msp430Adc12ImplP$ADCArbiterInfo$userId(void );
# 35 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void Msp430Adc12ImplP$ControlA1$setControl(msp430_compare_control_t control);
# 227 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static uint16_t * Msp430Adc12ImplP$SingleChannel$multipleDataReady(
# 41 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x408593f0, 
# 227 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t * buffer, uint16_t numSamples);
#line 206
static error_t Msp430Adc12ImplP$SingleChannel$singleDataReady(
# 41 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x408593f0, 
# 206 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t data);
# 64 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void Msp430Adc12ImplP$Port63$makeInput(void );
#line 85
static void Msp430Adc12ImplP$Port63$selectIOFunc(void );
#line 78
static void Msp430Adc12ImplP$Port63$selectModuleFunc(void );
# 30 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
static void Msp430Adc12ImplP$CompareA0$setEvent(uint16_t time);
# 64 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void Msp430Adc12ImplP$Port61$makeInput(void );
#line 85
static void Msp430Adc12ImplP$Port61$selectIOFunc(void );
#line 78
static void Msp430Adc12ImplP$Port61$selectModuleFunc(void );
#line 64
static void Msp430Adc12ImplP$Port66$makeInput(void );
#line 85
static void Msp430Adc12ImplP$Port66$selectIOFunc(void );
#line 78
static void Msp430Adc12ImplP$Port66$selectModuleFunc(void );
# 67 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
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

uint16_t Msp430Adc12ImplP$resultBufferLength;
uint16_t * Msp430Adc12ImplP$resultBufferStart;
uint16_t Msp430Adc12ImplP$resultBufferIndex;
uint8_t Msp430Adc12ImplP$numChannels;
uint8_t Msp430Adc12ImplP$clientID;

static inline error_t Msp430Adc12ImplP$Init$init(void );










static inline void Msp430Adc12ImplP$prepareTimerA(uint16_t interval, uint16_t csSAMPCON, uint16_t cdSAMPCON);
#line 117
static inline void Msp430Adc12ImplP$startTimerA(void );
#line 138
static inline void Msp430Adc12ImplP$configureAdcPin(uint8_t inch);
#line 155
static void Msp430Adc12ImplP$resetAdcPin(uint8_t inch);
#line 172
static error_t Msp430Adc12ImplP$SingleChannel$configureSingle(uint8_t id, 
const msp430adc12_channel_config_t *config);
#line 263
static inline error_t Msp430Adc12ImplP$SingleChannel$configureMultiple(uint8_t id, 
const msp430adc12_channel_config_t *config, 
uint16_t *buf, uint16_t length, uint16_t jiffies);
#line 378
static error_t Msp430Adc12ImplP$SingleChannel$getData(uint8_t id);
#line 483
static void Msp430Adc12ImplP$stopConversion(void );
#line 520
static inline void Msp430Adc12ImplP$TimerA$overflow(void );
static inline void Msp430Adc12ImplP$CompareA0$fired(void );
static inline void Msp430Adc12ImplP$CompareA1$fired(void );

static inline void Msp430Adc12ImplP$HplAdc12$conversionDone(uint16_t iv);
#line 620
static inline error_t Msp430Adc12ImplP$SingleChannel$default$singleDataReady(uint8_t id, uint16_t data);




static inline uint16_t *Msp430Adc12ImplP$SingleChannel$default$multipleDataReady(uint8_t id, 
uint16_t *buf, uint16_t numSamples);




static inline void Msp430Adc12ImplP$MultiChannel$default$dataReady(uint8_t id, uint16_t *buffer, uint16_t numSamples);

static inline void Msp430Adc12ImplP$Overflow$default$memOverflow(uint8_t id);
static inline void Msp430Adc12ImplP$Overflow$default$conversionTimeOverflow(uint8_t id);
# 112 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/HplAdc12.nc"
static void HplAdc12P$HplAdc12$conversionDone(uint16_t iv);
# 51 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/HplAdc12P.nc"
static volatile uint16_t HplAdc12P$ADC12CTL0 __asm ("0x01A0");
static volatile uint16_t HplAdc12P$ADC12CTL1 __asm ("0x01A2");
static volatile uint16_t HplAdc12P$ADC12IFG __asm ("0x01A4");
static volatile uint16_t HplAdc12P$ADC12IE __asm ("0x01A6");
static volatile uint16_t HplAdc12P$ADC12IV __asm ("0x01A8");





static inline void HplAdc12P$HplAdc12$setCtl0(adc12ctl0_t control0);



static inline void HplAdc12P$HplAdc12$setCtl1(adc12ctl1_t control1);



static inline adc12ctl0_t HplAdc12P$HplAdc12$getCtl0(void );







static inline void HplAdc12P$HplAdc12$setMCtl(uint8_t i, adc12memctl_t memControl);





static adc12memctl_t HplAdc12P$HplAdc12$getMCtl(uint8_t i);







static inline uint16_t HplAdc12P$HplAdc12$getMem(uint8_t i);



static inline void HplAdc12P$HplAdc12$setIEFlags(uint16_t mask);


static inline void HplAdc12P$HplAdc12$resetIFGs(void );




static inline void HplAdc12P$HplAdc12$startConversion(void );




static void HplAdc12P$HplAdc12$stopConversion(void );








static inline void HplAdc12P$HplAdc12$enableConversion(void );



static inline bool HplAdc12P$HplAdc12$isBusy(void );

void sig_ADC_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(14)))  ;
# 39 "/opt/tinyos-2.1.0/tos/system/RoundRobinResourceQueueC.nc"
enum /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$__nesc_unnamed4281 {
  RoundRobinResourceQueueC$0$NO_ENTRY = 0xFF, 
  RoundRobinResourceQueueC$0$SIZE = 2U ? (2U - 1) / 8 + 1 : 0
};

uint8_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$resQ[/*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$SIZE];
uint8_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$last = 0;

static inline void /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$clearEntry(uint8_t id);



static inline error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$Init$init(void );




static inline bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$isEmpty(void );








static bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$isEnqueued(resource_client_id_t id);



static inline resource_client_id_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$dequeue(void );
#line 87
static inline error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$enqueue(resource_client_id_t id);
# 43 "/opt/tinyos-2.1.0/tos/interfaces/ResourceRequested.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceRequested$requested(
# 52 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x40903318);
# 55 "/opt/tinyos-2.1.0/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$unconfigure(
# 56 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x40902010);
# 49 "/opt/tinyos-2.1.0/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$configure(
# 56 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x40902010);
# 69 "/opt/tinyos-2.1.0/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Queue$enqueue(resource_client_id_t id);
#line 43
static bool /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Queue$isEmpty(void );
#line 60
static resource_client_id_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Queue$dequeue(void );
# 92 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$granted(
# 51 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x409048e0);
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask$postTask(void );
# 69 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
enum /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$__nesc_unnamed4282 {
#line 69
  SimpleArbiterP$0$grantedTask = 1U
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



static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$request(uint8_t id);
#line 97
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$release(uint8_t id);
#line 136
static uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ArbiterInfo$userId(void );
#line 154
static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask$runTask(void );









static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$default$granted(uint8_t id);

static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceRequested$default$requested(uint8_t id);



static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$configure(uint8_t id);

static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$unconfigure(uint8_t id);
# 63 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/HplAdc12.nc"
static adc12ctl0_t Msp430RefVoltGeneratorP$HplAdc12$getCtl0(void );
#line 118
static bool Msp430RefVoltGeneratorP$HplAdc12$isBusy(void );
#line 51
static void Msp430RefVoltGeneratorP$HplAdc12$setCtl0(adc12ctl0_t control0);
# 62 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
static void Msp430RefVoltGeneratorP$SwitchOffTimer$startOneShot(uint32_t dt);




static void Msp430RefVoltGeneratorP$SwitchOffTimer$stop(void );
# 92 "/opt/tinyos-2.1.0/tos/interfaces/SplitControl.nc"
static void Msp430RefVoltGeneratorP$RefVolt_2_5V$startDone(error_t error);
#line 117
static void Msp430RefVoltGeneratorP$RefVolt_2_5V$stopDone(error_t error);
#line 92
static void Msp430RefVoltGeneratorP$RefVolt_1_5V$startDone(error_t error);
#line 117
static void Msp430RefVoltGeneratorP$RefVolt_1_5V$stopDone(error_t error);
# 62 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
static void Msp430RefVoltGeneratorP$SwitchOnTimer$startOneShot(uint32_t dt);




static void Msp430RefVoltGeneratorP$SwitchOnTimer$stop(void );
# 47 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
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
static error_t Msp430RefVoltGeneratorP$switchOff(void );
#line 94
static inline error_t Msp430RefVoltGeneratorP$RefVolt_1_5V$start(void );
#line 127
static inline error_t Msp430RefVoltGeneratorP$RefVolt_1_5V$stop(void );
#line 157
static inline error_t Msp430RefVoltGeneratorP$RefVolt_2_5V$start(void );
#line 220
static inline void Msp430RefVoltGeneratorP$SwitchOnTimer$fired(void );
#line 244
static inline void Msp430RefVoltGeneratorP$SwitchOffTimer$fired(void );
#line 274
static inline void Msp430RefVoltGeneratorP$HplAdc12$conversionDone(uint16_t iv);
# 58 "/opt/tinyos-2.1.0/tos/interfaces/AdcConfigure.nc"
static Msp430RefVoltArbiterImplP$Config$adc_config_t Msp430RefVoltArbiterImplP$Config$getConfiguration(
# 43 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x40968d10);
# 83 "/opt/tinyos-2.1.0/tos/interfaces/SplitControl.nc"
static error_t Msp430RefVoltArbiterImplP$RefVolt_2_5V$start(void );
# 110 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t Msp430RefVoltArbiterImplP$AdcResource$release(
# 40 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x409692b8);
# 78 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t Msp430RefVoltArbiterImplP$AdcResource$request(
# 40 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x409692b8);
# 92 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static void Msp430RefVoltArbiterImplP$ClientResource$granted(
# 38 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x409357d0);
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t Msp430RefVoltArbiterImplP$switchOff$postTask(void );
# 83 "/opt/tinyos-2.1.0/tos/interfaces/SplitControl.nc"
static error_t Msp430RefVoltArbiterImplP$RefVolt_1_5V$start(void );
#line 109
static error_t Msp430RefVoltArbiterImplP$RefVolt_1_5V$stop(void );
# 51 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
enum Msp430RefVoltArbiterImplP$__nesc_unnamed4286 {
#line 51
  Msp430RefVoltArbiterImplP$switchOff = 2U
};
#line 51
typedef int Msp430RefVoltArbiterImplP$__nesc_sillytask_switchOff[Msp430RefVoltArbiterImplP$switchOff];
#line 46
enum Msp430RefVoltArbiterImplP$__nesc_unnamed4287 {
  Msp430RefVoltArbiterImplP$NO_OWNER = 0xFF
};
uint8_t Msp430RefVoltArbiterImplP$syncOwner = Msp430RefVoltArbiterImplP$NO_OWNER;



static inline error_t Msp430RefVoltArbiterImplP$ClientResource$request(uint8_t client);
#line 70
static void Msp430RefVoltArbiterImplP$AdcResource$granted(uint8_t client);
#line 98
static inline void Msp430RefVoltArbiterImplP$RefVolt_1_5V$startDone(error_t error);








static void Msp430RefVoltArbiterImplP$RefVolt_2_5V$startDone(error_t error);








static error_t Msp430RefVoltArbiterImplP$ClientResource$release(uint8_t client);
#line 136
static inline void Msp430RefVoltArbiterImplP$switchOff$runTask(void );










static inline void Msp430RefVoltArbiterImplP$RefVolt_1_5V$stopDone(error_t error);



static inline void Msp430RefVoltArbiterImplP$RefVolt_2_5V$stopDone(error_t error);








static inline void Msp430RefVoltArbiterImplP$ClientResource$default$granted(uint8_t client);
static inline error_t Msp430RefVoltArbiterImplP$AdcResource$default$request(uint8_t client);








static inline error_t Msp430RefVoltArbiterImplP$AdcResource$default$release(uint8_t client);
const msp430adc12_channel_config_t Msp430RefVoltArbiterImplP$defaultConfig = { INPUT_CHANNEL_NONE, 0, 0, 0, 0, 0, 0, 0 };
static inline const msp430adc12_channel_config_t *
Msp430RefVoltArbiterImplP$Config$default$getConfiguration(uint8_t client);
# 30 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$setEvent(uint16_t time);

static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$setEventFromNow(uint16_t delta);
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$get(void );
# 67 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$fired(void );
# 46 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$enableEvents(void );
#line 36
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$setControlAsCompare(void );










static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$disableEvents(void );
#line 33
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$clearPendingInterrupt(void );
# 42 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Init$init(void );
#line 54
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$stop(void );




static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$fired(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$startAt(uint16_t t0, uint16_t dt);
#line 103
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$overflow(void );
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$get(void );
static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$isOverflowPending(void );
# 71 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$overflow(void );
# 38 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$get(void );




static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$isOverflowPending(void );









static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$overflow(void );
# 53 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
static /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$size_type /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$get(void );






static bool /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$overflow(void );
# 56 "/opt/tinyos-2.1.0/tos/lib/timer/TransformCounterC.nc"
/*CounterMilli32C.Transform*/TransformCounterC$0$upper_count_type /*CounterMilli32C.Transform*/TransformCounterC$0$m_upper;

enum /*CounterMilli32C.Transform*/TransformCounterC$0$__nesc_unnamed4288 {

  TransformCounterC$0$LOW_SHIFT_RIGHT = 5, 
  TransformCounterC$0$HIGH_SHIFT_LEFT = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC$0$from_size_type ) - /*CounterMilli32C.Transform*/TransformCounterC$0$LOW_SHIFT_RIGHT, 
  TransformCounterC$0$NUM_UPPER_BITS = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type ) - 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC$0$from_size_type ) + 5, 



  TransformCounterC$0$OVERFLOW_MASK = /*CounterMilli32C.Transform*/TransformCounterC$0$NUM_UPPER_BITS ? ((/*CounterMilli32C.Transform*/TransformCounterC$0$upper_count_type )2 << (/*CounterMilli32C.Transform*/TransformCounterC$0$NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$get(void );
#line 122
static inline void /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$overflow(void );
# 67 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$fired(void );
#line 92
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$size_type dt);
#line 62
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$stop(void );
# 53 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$get(void );
# 66 "/opt/tinyos-2.1.0/tos/lib/timer/TransformAlarmC.nc"
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0;
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_dt;

enum /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$__nesc_unnamed4289 {

  TransformAlarmC$0$MAX_DELAY_LOG2 = 8 * sizeof(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$from_size_type ) - 1 - 5, 
  TransformAlarmC$0$MAX_DELAY = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type )1 << /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$MAX_DELAY_LOG2
};

static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getNow(void );




static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getAlarm(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$set_alarm(void );
#line 136
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type dt);
#line 151
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$fired(void );
#line 166
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$overflow(void );
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$postTask(void );
# 98 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$getNow(void );
#line 92
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type dt);
#line 105
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$getAlarm(void );
#line 62
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$stop(void );
# 72 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$fired(void );
# 63 "/opt/tinyos-2.1.0/tos/lib/timer/AlarmToTimerC.nc"
enum /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$__nesc_unnamed4290 {
#line 63
  AlarmToTimerC$0$fired = 3U
};
#line 63
typedef int /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$__nesc_sillytask_fired[/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired];
#line 44
uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_dt;
bool /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_oneshot;

static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$start(uint32_t t0, uint32_t dt, bool oneshot);
#line 60
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$stop(void );


static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$runTask(void );






static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$fired(void );
#line 82
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$startOneShotAt(uint32_t t0, uint32_t dt);


static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$getNow(void );
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$postTask(void );
# 125 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow(void );
#line 118
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$startOneShotAt(uint32_t t0, uint32_t dt);
#line 67
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$stop(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$fired(
# 37 "/opt/tinyos-2.1.0/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x40a2b9f0);
#line 60
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$__nesc_unnamed4291 {
#line 60
  VirtualizeTimerC$0$updateFromTimer = 4U
};
#line 60
typedef int /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$__nesc_sillytask_updateFromTimer[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer];
#line 42
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$__nesc_unnamed4292 {

  VirtualizeTimerC$0$NUM_TIMERS = 3U, 
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
#line 89
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$runTask(void );
#line 128
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$fired(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot);









static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startPeriodic(uint8_t num, uint32_t dt);




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShot(uint8_t num, uint32_t dt);




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$stop(uint8_t num);
#line 193
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$default$fired(uint8_t num);
# 47 "/opt/tinyos-2.1.0/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$overflow(void );
# 58 "/opt/tinyos-2.1.0/tos/interfaces/AdcConfigure.nc"
static /*AntiTheftAppC.ParLight.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$0$ConfUp$adc_config_t /*AntiTheftAppC.ParLight.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$0$ConfUp$getConfiguration(void );
# 47 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline const msp430adc12_channel_config_t */*AntiTheftAppC.ParLight.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$0$ConfSub$getConfiguration(void );
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t AdcStreamP$bufferDone$postTask(void );
#line 56
static error_t AdcStreamP$readStreamDone$postTask(void );
#line 56
static error_t AdcStreamP$readStreamFail$postTask(void );
# 98 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
static AdcStreamP$Alarm$size_type AdcStreamP$Alarm$getNow(void );
#line 92
static void AdcStreamP$Alarm$startAt(AdcStreamP$Alarm$size_type t0, AdcStreamP$Alarm$size_type dt);
# 58 "/opt/tinyos-2.1.0/tos/interfaces/AdcConfigure.nc"
static AdcStreamP$AdcConfigure$adc_config_t AdcStreamP$AdcConfigure$getConfiguration(
# 53 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x40aa7428);
# 189 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t AdcStreamP$SingleChannel$getData(
# 52 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x40aa8598);
# 84 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t AdcStreamP$SingleChannel$configureSingle(
# 52 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x40aa8598, 
# 84 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t * config);
#line 138
static error_t AdcStreamP$SingleChannel$configureMultiple(
# 52 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x40aa8598, 
# 138 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t * config, uint16_t * buffer, uint16_t numSamples, uint16_t jiffies);
# 89 "/opt/tinyos-2.1.0/tos/interfaces/ReadStream.nc"
static void AdcStreamP$ReadStream$bufferDone(
# 49 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x40a869f0, 
# 89 "/opt/tinyos-2.1.0/tos/interfaces/ReadStream.nc"
error_t result, 
#line 86
AdcStreamP$ReadStream$val_t * buf, 



uint16_t count);
#line 102
static void AdcStreamP$ReadStream$readDone(
# 49 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x40a869f0, 
# 102 "/opt/tinyos-2.1.0/tos/interfaces/ReadStream.nc"
error_t result, uint32_t usActualPeriod);
# 114 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcStreamP.nc"
enum AdcStreamP$__nesc_unnamed4294 {
#line 114
  AdcStreamP$readStreamDone = 5U
};
#line 114
typedef int AdcStreamP$__nesc_sillytask_readStreamDone[AdcStreamP$readStreamDone];
#line 130
enum AdcStreamP$__nesc_unnamed4295 {
#line 130
  AdcStreamP$readStreamFail = 6U
};
#line 130
typedef int AdcStreamP$__nesc_sillytask_readStreamFail[AdcStreamP$readStreamFail];
#line 151
enum AdcStreamP$__nesc_unnamed4296 {
#line 151
  AdcStreamP$bufferDone = 7U
};
#line 151
typedef int AdcStreamP$__nesc_sillytask_bufferDone[AdcStreamP$bufferDone];
#line 58
enum AdcStreamP$__nesc_unnamed4297 {
  AdcStreamP$NSTREAM = 1U
};




uint8_t AdcStreamP$client = AdcStreamP$NSTREAM;


struct AdcStreamP$list_entry_t {
  uint16_t count;
  struct AdcStreamP$list_entry_t * next;
};
struct AdcStreamP$list_entry_t *AdcStreamP$bufferQueue[AdcStreamP$NSTREAM];
struct AdcStreamP$list_entry_t * *AdcStreamP$bufferQueueEnd[AdcStreamP$NSTREAM];
uint16_t * AdcStreamP$lastBuffer;
#line 74
uint16_t AdcStreamP$lastCount;

uint16_t AdcStreamP$count;
uint16_t * AdcStreamP$buffer;
uint16_t * AdcStreamP$pos;
uint32_t AdcStreamP$now;
#line 79
uint32_t AdcStreamP$period;
bool AdcStreamP$periodModified;


static inline error_t AdcStreamP$Init$init(void );








static inline void AdcStreamP$sampleSingle(void );
#line 114
static inline void AdcStreamP$readStreamDone$runTask(void );
#line 130
static inline void AdcStreamP$readStreamFail$runTask(void );
#line 151
static inline void AdcStreamP$bufferDone$runTask(void );
#line 163
static inline void AdcStreamP$nextAlarm(void );




static inline void AdcStreamP$Alarm$fired(void );



static error_t AdcStreamP$nextBuffer(bool startNextAlarm);
#line 201
static void AdcStreamP$nextMultiple(uint8_t c);










static error_t AdcStreamP$ReadStream$read(uint8_t c, uint32_t usPeriod);
#line 233
static error_t AdcStreamP$SingleChannel$singleDataReady(uint8_t streamClient, uint16_t data);
#line 272
static uint16_t *AdcStreamP$SingleChannel$multipleDataReady(uint8_t streamClient, 
uint16_t *buf, uint16_t length);
#line 295
const msp430adc12_channel_config_t AdcStreamP$defaultConfig = { 
.inch = SUPPLY_VOLTAGE_HALF_CHANNEL, 
.sref = REFERENCE_VREFplus_AVss, 
.ref2_5v = REFVOLT_LEVEL_1_5, 
.adc12ssel = SHT_SOURCE_ACLK, 
.adc12div = SHT_CLOCK_DIV_1, 
.sht = SAMPLE_HOLD_4_CYCLES, 
.sampcon_ssel = SAMPCON_SOURCE_SMCLK, 
.sampcon_id = SAMPCON_CLOCK_DIV_1 };

static inline const msp430adc12_channel_config_t *AdcStreamP$AdcConfigure$default$getConfiguration(uint8_t c);



static inline error_t AdcStreamP$SingleChannel$default$configureMultiple(uint8_t c, 
const msp430adc12_channel_config_t *config, uint16_t b[], 
uint16_t numSamples, uint16_t jiffies);



static inline error_t AdcStreamP$SingleChannel$default$getData(uint8_t c);



static inline error_t AdcStreamP$SingleChannel$default$configureSingle(uint8_t c, 
const msp430adc12_channel_config_t *config);
# 30 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Msp430Compare$setEvent(uint16_t time);

static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Msp430Compare$setEventFromNow(uint16_t delta);
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Msp430Timer$get(void );
# 67 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Alarm$fired(void );
# 46 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Msp430TimerControl$enableEvents(void );
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Msp430TimerControl$disableEvents(void );
#line 33
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Msp430TimerControl$clearPendingInterrupt(void );
# 59 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Msp430Compare$fired(void );










static inline void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Alarm$startAt(uint16_t t0, uint16_t dt);
#line 103
static inline void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Msp430Timer$overflow(void );
# 67 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$Alarm$fired(void );
#line 92
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$AlarmFrom$startAt(/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$AlarmFrom$size_type t0, /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$AlarmFrom$size_type dt);
# 53 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
static /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$Counter$size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$Counter$get(void );
# 66 "/opt/tinyos-2.1.0/tos/lib/timer/TransformAlarmC.nc"
/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$to_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$m_t0;
/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$to_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$m_dt;

enum /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$__nesc_unnamed4298 {

  TransformAlarmC$1$MAX_DELAY_LOG2 = 8 * sizeof(/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$from_size_type ) - 1 - 5, 
  TransformAlarmC$1$MAX_DELAY = (/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$to_size_type )1 << /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$MAX_DELAY_LOG2
};

static inline /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$to_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$Alarm$getNow(void );
#line 96
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$set_alarm(void );
#line 136
static inline void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$Alarm$startAt(/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$to_size_type t0, /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$to_size_type dt);
#line 151
static inline void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$AlarmFrom$fired(void );
#line 166
static inline void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$Counter$overflow(void );
# 78 "/opt/tinyos-2.1.0/tos/interfaces/ReadStream.nc"
static error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$Service$read(
# 26 "/opt/tinyos-2.1.0/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x40adad00, 
# 78 "/opt/tinyos-2.1.0/tos/interfaces/ReadStream.nc"
uint32_t usPeriod);










static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$ReadStream$bufferDone(
# 24 "/opt/tinyos-2.1.0/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x40adb258, 
# 89 "/opt/tinyos-2.1.0/tos/interfaces/ReadStream.nc"
error_t result, 
#line 86
/*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$ReadStream$val_t * buf, 



uint16_t count);
#line 102
static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$ReadStream$readDone(
# 24 "/opt/tinyos-2.1.0/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x40adb258, 
# 102 "/opt/tinyos-2.1.0/tos/interfaces/ReadStream.nc"
error_t result, uint32_t usActualPeriod);
# 110 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$Resource$release(
# 27 "/opt/tinyos-2.1.0/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x40ad7678);



uint32_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$period[1U];
#line 48
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$Service$bufferDone(uint8_t client, error_t result, /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$val_t *buf, uint16_t count);




static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$Service$readDone(uint8_t client, error_t result, uint32_t actualPeriod);





static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$Resource$granted(uint8_t client);







static inline error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$Resource$default$release(uint8_t client);
#line 79
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$ReadStream$default$bufferDone(uint8_t client, error_t result, /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$val_t *buf, uint16_t count);



static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$ReadStream$default$readDone(uint8_t client, error_t result, uint32_t actualPeriod);
# 58 "/opt/tinyos-2.1.0/tos/interfaces/AdcConfigure.nc"
static /*AntiTheftAppC.ParLight.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$1$ConfUp$adc_config_t /*AntiTheftAppC.ParLight.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$1$ConfUp$getConfiguration(void );
# 47 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline const msp430adc12_channel_config_t */*AntiTheftAppC.ParLight.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$1$ConfSub$getConfiguration(void );
# 48 "/opt/tinyos-2.1.0/tos/platforms/telosa/chips/s1087/HamamatsuS1087ParP.nc"
msp430adc12_channel_config_t HamamatsuS1087ParP$config = { 
.inch = INPUT_CHANNEL_A4, 
.sref = REFERENCE_VREFplus_AVss, 
.ref2_5v = REFVOLT_LEVEL_1_5, 
.adc12ssel = SHT_SOURCE_ACLK, 
.adc12div = SHT_CLOCK_DIV_1, 
.sht = SAMPLE_HOLD_4_CYCLES, 
.sampcon_ssel = SAMPCON_SOURCE_SMCLK, 
.sampcon_id = SAMPCON_CLOCK_DIV_1 };




static inline const msp430adc12_channel_config_t *HamamatsuS1087ParP$AdcConfigure$getConfiguration(void );
# 212 "/opt/tinyos-2.1.0/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_enable_interrupt(void )
{
   __asm volatile ("eint");}

# 185 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Timer$overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Timer$overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Timer$overflow(void )
{
}

# 520 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP$TimerA$overflow(void )
#line 520
{
}

# 37 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$overflow(void ){
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
# 126 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Overflow$fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$overflow();
}





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$default$fired(uint8_t n)
{
}

# 28 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$fired(uint8_t arg_0x40615428){
#line 28
  switch (arg_0x40615428) {
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
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$default$fired(arg_0x40615428);
#line 28
      break;
#line 28
    }
#line 28
}
#line 28
# 115 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX0$fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$fired(0);
}

# 28 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP$VectorTimerA0$fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX0$fired();
#line 28
}
#line 28
# 47 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$__nesc_unnamed4299 {
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
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Control$getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$int2CC(* (volatile uint16_t * )354U);
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$default$captured(time);
#line 75
}
#line 75
# 139 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$getEvent(void )
{
  return * (volatile uint16_t * )370U;
}

# 521 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP$CompareA0$fired(void )
#line 521
{
}

# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$fired(void ){
#line 34
  Msp430Adc12ImplP$CompareA0$fired();
#line 34
}
#line 34
# 47 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$__nesc_unnamed4300 {
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
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Control$getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$int2CC(* (volatile uint16_t * )356U);
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$default$captured(time);
#line 75
}
#line 75
# 139 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$getEvent(void )
{
  return * (volatile uint16_t * )372U;
}

# 522 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP$CompareA1$fired(void )
#line 522
{
}

# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$fired(void ){
#line 34
  Msp430Adc12ImplP$CompareA1$fired();
#line 34
}
#line 34
# 47 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$__nesc_unnamed4301 {
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
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Control$getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$int2CC(* (volatile uint16_t * )358U);
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$default$captured(time);
#line 75
}
#line 75
# 139 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$getEvent(void )
{
  return * (volatile uint16_t * )374U;
}

#line 181
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$default$fired(void )
{
}

# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$default$fired();
#line 34
}
#line 34
# 120 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX1$fired(void )
{
  uint8_t n = * (volatile uint16_t * )302U;

#line 123
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$fired(n >> 1);
}

# 28 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP$VectorTimerA1$fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX1$fired();
#line 28
}
#line 28
# 115 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX0$fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$fired(0);
}

# 28 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP$VectorTimerB0$fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX0$fired();
#line 28
}
#line 28
# 185 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Timer$overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Timer$overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Timer$overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Timer$overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Timer$overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Timer$overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$overflow(void )
{
}

# 103 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Msp430Timer$overflow(void )
{
}

#line 103
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$overflow(void )
{
}

# 47 "/opt/tinyos-2.1.0/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$overflow(void )
{
}

# 166 "/opt/tinyos-2.1.0/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$overflow(void )
{
}

#line 166
static inline void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$Counter$overflow(void )
{
}

# 71 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
inline static void /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$overflow(void ){
#line 71
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$Counter$overflow();
#line 71
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$overflow();
#line 71
  /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$overflow();
#line 71
}
#line 71
# 122 "/opt/tinyos-2.1.0/tos/lib/timer/TransformCounterC.nc"
static inline void /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*CounterMilli32C.Transform*/TransformCounterC$0$m_upper++;
    if ((/*CounterMilli32C.Transform*/TransformCounterC$0$m_upper & /*CounterMilli32C.Transform*/TransformCounterC$0$OVERFLOW_MASK) == 0) {
      /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$overflow();
      }
  }
}

# 71 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
inline static void /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$overflow(void ){
#line 71
  /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$overflow();
#line 71
}
#line 71
# 53 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$overflow(void )
{
  /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$overflow();
}

# 37 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$overflow(void ){
#line 37
  /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$overflow();
#line 37
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$overflow();
#line 37
  /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Msp430Timer$overflow();
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
# 126 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Overflow$fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$overflow();
}

# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$postTask(void ){
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
# 70 "/opt/tinyos-2.1.0/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$fired(void )
{
#line 71
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$postTask();
}

# 67 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$fired(void ){
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$fired();
#line 67
}
#line 67
# 151 "/opt/tinyos-2.1.0/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$fired(void )
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

# 67 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$fired(void ){
#line 67
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$fired();
#line 67
}
#line 67
# 124 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$disableEvents(void )
{
  * (volatile uint16_t * )386U &= ~0x0010;
}

# 47 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$disableEvents(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$disableEvents();
#line 47
}
#line 47
# 59 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$fired(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$fired();
}

# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$fired(void ){
#line 34
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$getEvent(void )
{
  return * (volatile uint16_t * )402U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$default$captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$__nesc_unnamed4302 {
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
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$int2CC(* (volatile uint16_t * )386U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Event$fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$captured(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$fired();
    }
}

# 86 "/opt/tinyos-2.1.0/tos/system/SchedulerBasicP.nc"
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

# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$get(void ){
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
# 38 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$get(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$get();
}

# 53 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
inline static /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$size_type /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$get(void ){
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
# 70 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$isOverflowPending(void )
{
  return * (volatile uint16_t * )384U & 1U;
}

# 35 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
inline static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$isOverflowPending(void ){
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
# 43 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$isOverflowPending(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$isOverflowPending();
}

# 60 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
inline static bool /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$isOverflowPending(void ){
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
# 119 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$enableEvents(void )
{
  * (volatile uint16_t * )386U |= 0x0010;
}

# 46 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$enableEvents(void ){
#line 46
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$enableEvents();
#line 46
}
#line 46
# 84 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$clearPendingInterrupt(void )
{
  * (volatile uint16_t * )386U &= ~0x0001;
}

# 33 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$clearPendingInterrupt(void ){
#line 33
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$clearPendingInterrupt();
#line 33
}
#line 33
# 144 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEvent(uint16_t x)
{
  * (volatile uint16_t * )402U = x;
}

# 30 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$setEvent(uint16_t time){
#line 30
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEvent(time);
#line 30
}
#line 30
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$get(void ){
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
# 154 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )402U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$get() + x;
}

# 32 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$setEventFromNow(uint16_t delta){
#line 32
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEventFromNow(delta);
#line 32
}
#line 32
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$get(void ){
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
# 70 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$startAt(uint16_t t0, uint16_t dt)
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

# 92 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$size_type dt){
#line 92
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$startAt(t0, dt);
#line 92
}
#line 92
# 181 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$default$fired(void )
{
}

# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$default$fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$getEvent(void )
{
  return * (volatile uint16_t * )404U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$default$captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$__nesc_unnamed4303 {
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
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$int2CC(* (volatile uint16_t * )388U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Event$fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$captured(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$fired();
    }
}

# 315 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline error_t AdcStreamP$SingleChannel$default$getData(uint8_t c)
{
  return FAIL;
}

# 189 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static error_t AdcStreamP$SingleChannel$getData(uint8_t arg_0x40aa8598){
#line 189
  unsigned char result;
#line 189

#line 189
  switch (arg_0x40aa8598) {
#line 189
    case /*AntiTheftAppC.ParLight.AdcReadStreamClientC*/AdcReadStreamClientC$0$RSCLIENT:
#line 189
      result = Msp430Adc12ImplP$SingleChannel$getData(/*AntiTheftAppC.ParLight.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$1$ID);
#line 189
      break;
#line 189
    default:
#line 189
      result = AdcStreamP$SingleChannel$default$getData(arg_0x40aa8598);
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
# 92 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline void AdcStreamP$sampleSingle(void )
#line 92
{
  AdcStreamP$SingleChannel$getData(AdcStreamP$client);
}

#line 168
static inline void AdcStreamP$Alarm$fired(void )
#line 168
{
  AdcStreamP$sampleSingle();
}

# 67 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
inline static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$Alarm$fired(void ){
#line 67
  AdcStreamP$Alarm$fired();
#line 67
}
#line 67
# 151 "/opt/tinyos-2.1.0/tos/lib/timer/TransformAlarmC.nc"
static inline void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$AlarmFrom$fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$m_dt == 0) 
      {
        /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$Alarm$fired();
      }
    else 
      {
        /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$set_alarm();
      }
  }
}

# 67 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
inline static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Alarm$fired(void ){
#line 67
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$AlarmFrom$fired();
#line 67
}
#line 67
# 124 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$disableEvents(void )
{
  * (volatile uint16_t * )390U &= ~0x0010;
}

# 47 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Msp430TimerControl$disableEvents(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$disableEvents();
#line 47
}
#line 47
# 59 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Msp430Compare$fired(void )
{
  /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Msp430TimerControl$disableEvents();
  /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Alarm$fired();
}

# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$fired(void ){
#line 34
  /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Msp430Compare$fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$getEvent(void )
{
  return * (volatile uint16_t * )406U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$default$captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$__nesc_unnamed4304 {
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
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$int2CC(* (volatile uint16_t * )390U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Event$fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$captured(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$fired();
    }
}

# 50 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )54U &= ~(0x01 << 7);
}

# 64 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP$Port67$makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$makeInput();
#line 64
}
#line 64
# 54 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )55U |= 0x01 << 7;
}

# 78 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP$Port67$selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$selectModuleFunc();
#line 78
}
#line 78
# 50 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )54U &= ~(0x01 << 6);
}

# 64 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP$Port66$makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$makeInput();
#line 64
}
#line 64
# 54 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )55U |= 0x01 << 6;
}

# 78 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP$Port66$selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$selectModuleFunc();
#line 78
}
#line 78
# 50 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )54U &= ~(0x01 << 5);
}

# 64 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP$Port65$makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$makeInput();
#line 64
}
#line 64
# 54 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )55U |= 0x01 << 5;
}

# 78 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP$Port65$selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$selectModuleFunc();
#line 78
}
#line 78
# 50 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )54U &= ~(0x01 << 4);
}

# 64 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP$Port64$makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$makeInput();
#line 64
}
#line 64
# 54 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )55U |= 0x01 << 4;
}

# 78 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP$Port64$selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$selectModuleFunc();
#line 78
}
#line 78
# 50 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )54U &= ~(0x01 << 3);
}

# 64 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP$Port63$makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$makeInput();
#line 64
}
#line 64
# 54 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )55U |= 0x01 << 3;
}

# 78 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP$Port63$selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$selectModuleFunc();
#line 78
}
#line 78
# 50 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )54U &= ~(0x01 << 2);
}

# 64 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP$Port62$makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$makeInput();
#line 64
}
#line 64
# 54 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )55U |= 0x01 << 2;
}

# 78 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP$Port62$selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$selectModuleFunc();
#line 78
}
#line 78
# 50 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )54U &= ~(0x01 << 1);
}

# 64 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP$Port61$makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$makeInput();
#line 64
}
#line 64
# 54 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )55U |= 0x01 << 1;
}

# 78 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP$Port61$selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$selectModuleFunc();
#line 78
}
#line 78
# 50 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )54U &= ~(0x01 << 0);
}

# 64 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP$Port60$makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$makeInput();
#line 64
}
#line 64
# 54 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )55U |= 0x01 << 0;
}

# 78 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP$Port60$selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$selectModuleFunc();
#line 78
}
#line 78
# 138 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP$configureAdcPin(uint8_t inch)
{

  switch (inch) 
    {
      case 0: Msp430Adc12ImplP$Port60$selectModuleFunc();
#line 143
      Msp430Adc12ImplP$Port60$makeInput();
#line 143
      break;
      case 1: Msp430Adc12ImplP$Port61$selectModuleFunc();
#line 144
      Msp430Adc12ImplP$Port61$makeInput();
#line 144
      break;
      case 2: Msp430Adc12ImplP$Port62$selectModuleFunc();
#line 145
      Msp430Adc12ImplP$Port62$makeInput();
#line 145
      break;
      case 3: Msp430Adc12ImplP$Port63$selectModuleFunc();
#line 146
      Msp430Adc12ImplP$Port63$makeInput();
#line 146
      break;
      case 4: Msp430Adc12ImplP$Port64$selectModuleFunc();
#line 147
      Msp430Adc12ImplP$Port64$makeInput();
#line 147
      break;
      case 5: Msp430Adc12ImplP$Port65$selectModuleFunc();
#line 148
      Msp430Adc12ImplP$Port65$makeInput();
#line 148
      break;
      case 6: Msp430Adc12ImplP$Port66$selectModuleFunc();
#line 149
      Msp430Adc12ImplP$Port66$makeInput();
#line 149
      break;
      case 7: Msp430Adc12ImplP$Port67$selectModuleFunc();
#line 150
      Msp430Adc12ImplP$Port67$makeInput();
#line 150
      break;
    }
}

# 103 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline void HplAdc12P$HplAdc12$startConversion(void )
#line 103
{
  HplAdc12P$ADC12CTL0 |= 0x0010;
  HplAdc12P$ADC12CTL0 |= 0x0001 + 0x0002;
}

# 128 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/HplAdc12.nc"
inline static void Msp430Adc12ImplP$HplAdc12$startConversion(void ){
#line 128
  HplAdc12P$HplAdc12$startConversion();
#line 128
}
#line 128
# 39 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void Msp430Adc12ImplP$TimerA$setMode(int mode){
#line 39
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setMode(mode);
#line 39
}
#line 39
# 46 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$CC2int(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t x)
#line 46
{
#line 46
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$__nesc_unnamed4305 {
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
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Control$setControl(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t x)
{
  * (volatile uint16_t * )356U = /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$CC2int(x);
}

# 35 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void Msp430Adc12ImplP$ControlA1$setControl(msp430_compare_control_t control){
#line 35
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Control$setControl(control);
#line 35
}
#line 35
# 117 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP$startTimerA(void )
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

# 119 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$enableEvents(void )
{
  * (volatile uint16_t * )390U |= 0x0010;
}

# 46 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Msp430TimerControl$enableEvents(void ){
#line 46
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$enableEvents();
#line 46
}
#line 46
# 84 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$clearPendingInterrupt(void )
{
  * (volatile uint16_t * )390U &= ~0x0001;
}

# 33 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Msp430TimerControl$clearPendingInterrupt(void ){
#line 33
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$clearPendingInterrupt();
#line 33
}
#line 33
# 144 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$setEvent(uint16_t x)
{
  * (volatile uint16_t * )406U = x;
}

# 30 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Msp430Compare$setEvent(uint16_t time){
#line 30
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$setEvent(time);
#line 30
}
#line 30
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Timer$get(void ){
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
# 154 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )406U = /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Timer$get() + x;
}

# 32 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Msp430Compare$setEventFromNow(uint16_t delta){
#line 32
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$setEventFromNow(delta);
#line 32
}
#line 32
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Msp430Timer$get(void ){
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
# 70 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Alarm$startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Msp430Timer$get();
    uint16_t elapsed = now - t0;

#line 76
    if (elapsed >= dt) 
      {
        /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Msp430Compare$setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 83
        if (remaining <= 2) {
          /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Msp430Compare$setEventFromNow(2);
          }
        else {
#line 86
          /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Msp430Compare$setEvent(now + remaining);
          }
      }
#line 88
    /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Msp430TimerControl$clearPendingInterrupt();
    /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Msp430TimerControl$enableEvents();
  }
}

# 92 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
inline static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$AlarmFrom$startAt(/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$AlarmFrom$size_type t0, /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$AlarmFrom$size_type dt){
#line 92
  /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC$1$Alarm$startAt(t0, dt);
#line 92
}
#line 92
# 181 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$default$fired(void )
{
}

# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$default$fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$getEvent(void )
{
  return * (volatile uint16_t * )408U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$default$captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$__nesc_unnamed4306 {
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
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Control$getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$int2CC(* (volatile uint16_t * )392U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Event$fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$captured(/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$default$fired(void )
{
}

# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$default$fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$getEvent(void )
{
  return * (volatile uint16_t * )410U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$default$captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$__nesc_unnamed4307 {
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
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Control$getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$int2CC(* (volatile uint16_t * )394U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Event$fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$captured(/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$default$fired(void )
{
}

# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$default$fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$getEvent(void )
{
  return * (volatile uint16_t * )412U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$default$captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$__nesc_unnamed4308 {
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
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Control$getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$int2CC(* (volatile uint16_t * )396U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Event$fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$captured(/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$default$fired(void )
{
}

# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$default$fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$getEvent(void )
{
  return * (volatile uint16_t * )414U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$default$captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$__nesc_unnamed4309 {
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
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Control$getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$int2CC(* (volatile uint16_t * )398U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Event$fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$captured(/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$fired();
    }
}

# 120 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX1$fired(void )
{
  uint8_t n = * (volatile uint16_t * )286U;

#line 123
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$fired(n >> 1);
}

# 28 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP$VectorTimerB1$fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX1$fired();
#line 28
}
#line 28
# 113 "/opt/tinyos-2.1.0/tos/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP$Scheduler$init(void )
{
  /* atomic removed: atomic calls only */
  {
    memset((void *)SchedulerBasicP$m_next, SchedulerBasicP$NO_TASK, sizeof SchedulerBasicP$m_next);
    SchedulerBasicP$m_head = SchedulerBasicP$NO_TASK;
    SchedulerBasicP$m_tail = SchedulerBasicP$NO_TASK;
  }
}

# 46 "/opt/tinyos-2.1.0/tos/interfaces/Scheduler.nc"
inline static void RealMainP$Scheduler$init(void ){
#line 46
  SchedulerBasicP$Scheduler$init();
#line 46
}
#line 46
# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$set(void ){
#line 34
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$set();
#line 34
}
#line 34
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$set(void )
#line 37
{
#line 37
  /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$set();
}

# 29 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led2$set(void ){
#line 29
  /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$set();
#line 29
}
#line 29
# 45 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$set(void )
#line 45
{
  /* atomic removed: atomic calls only */
#line 45
  * (volatile uint8_t * )49U |= 0x01 << 5;
}

# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$set(void ){
#line 34
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$set();
#line 34
}
#line 34
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$set(void )
#line 37
{
#line 37
  /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$set();
}

# 29 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led1$set(void ){
#line 29
  /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$set();
#line 29
}
#line 29
# 45 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$set(void )
#line 45
{
  /* atomic removed: atomic calls only */
#line 45
  * (volatile uint8_t * )49U |= 0x01 << 4;
}

# 34 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$set(void ){
#line 34
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$set();
#line 34
}
#line 34
# 37 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$set(void )
#line 37
{
#line 37
  /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$set();
}

# 29 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led0$set(void ){
#line 29
  /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$set();
#line 29
}
#line 29
# 52 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )50U |= 0x01 << 6;
}

# 71 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$makeOutput(void )
#line 43
{
#line 43
  /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$makeOutput();
}

# 35 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led2$makeOutput(void ){
#line 35
  /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$makeOutput();
#line 35
}
#line 35
# 52 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )50U |= 0x01 << 5;
}

# 71 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$makeOutput(void )
#line 43
{
#line 43
  /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$makeOutput();
}

# 35 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led1$makeOutput(void ){
#line 35
  /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$makeOutput();
#line 35
}
#line 35
# 52 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )50U |= 0x01 << 4;
}

# 71 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$makeOutput(void )
#line 43
{
#line 43
  /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$makeOutput();
}

# 35 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led0$makeOutput(void ){
#line 35
  /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$makeOutput();
#line 35
}
#line 35
# 45 "/opt/tinyos-2.1.0/tos/system/LedsP.nc"
static inline error_t LedsP$Init$init(void )
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

# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
inline static error_t PlatformP$LedsInit$init(void ){
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
# 33 "/opt/tinyos-2.1.0/tos/platforms/telosb/hardware.h"
static inline  void TOSH_SET_SIMO0_PIN()
#line 33
{
#line 33
  static volatile uint8_t r __asm ("0x0019");

#line 33
  r |= 1 << 1;
}

#line 34
static inline  void TOSH_SET_UCLK0_PIN()
#line 34
{
#line 34
  static volatile uint8_t r __asm ("0x0019");

#line 34
  r |= 1 << 3;
}

#line 85
static inline  void TOSH_SET_FLASH_CS_PIN()
#line 85
{
#line 85
  static volatile uint8_t r __asm ("0x001D");

#line 85
  r |= 1 << 4;
}

#line 34
static inline  void TOSH_CLR_UCLK0_PIN()
#line 34
{
#line 34
  static volatile uint8_t r __asm ("0x0019");

#line 34
  r &= ~(1 << 3);
}

#line 85
static inline  void TOSH_CLR_FLASH_CS_PIN()
#line 85
{
#line 85
  static volatile uint8_t r __asm ("0x001D");

#line 85
  r &= ~(1 << 4);
}

# 11 "/opt/tinyos-2.1.0/tos/platforms/telosb/MotePlatformC.nc"
static __inline void MotePlatformC$TOSH_wait(void )
#line 11
{
   __asm volatile ("nop"); __asm volatile ("nop");}

# 86 "/opt/tinyos-2.1.0/tos/platforms/telosb/hardware.h"
static inline  void TOSH_SET_FLASH_HOLD_PIN()
#line 86
{
#line 86
  static volatile uint8_t r __asm ("0x001D");

#line 86
  r |= 1 << 7;
}

#line 85
static inline  void TOSH_MAKE_FLASH_CS_OUTPUT()
#line 85
{
#line 85
  static volatile uint8_t r __asm ("0x001E");

#line 85
  r |= 1 << 4;
}

#line 86
static inline  void TOSH_MAKE_FLASH_HOLD_OUTPUT()
#line 86
{
#line 86
  static volatile uint8_t r __asm ("0x001E");

#line 86
  r |= 1 << 7;
}

#line 34
static inline  void TOSH_MAKE_UCLK0_OUTPUT()
#line 34
{
#line 34
  static volatile uint8_t r __asm ("0x001A");

#line 34
  r |= 1 << 3;
}

#line 33
static inline  void TOSH_MAKE_SIMO0_OUTPUT()
#line 33
{
#line 33
  static volatile uint8_t r __asm ("0x001A");

#line 33
  r |= 1 << 1;
}

# 27 "/opt/tinyos-2.1.0/tos/platforms/telosb/MotePlatformC.nc"
static inline void MotePlatformC$TOSH_FLASH_M25P_DP(void )
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
  TOSH_SET_UCLK0_PIN();
  TOSH_SET_SIMO0_PIN();
}

#line 6
static __inline void MotePlatformC$uwait(uint16_t u)
#line 6
{
  uint16_t t0 = TA0R;

#line 8
  while (TA0R - t0 <= u) ;
}

#line 56
static inline error_t MotePlatformC$Init$init(void )
#line 56
{
  /* atomic removed: atomic calls only */

  {
    P1SEL = 0;
    P2SEL = 0;
    P3SEL = 0;
    P4SEL = 0;
    P5SEL = 0;
    P6SEL = 0;

    P1OUT = 0x00;
    P1DIR = 0xe0;

    P2OUT = 0x30;
    P2DIR = 0x7b;

    P3OUT = 0x00;
    P3DIR = 0xf1;

    P4OUT = 0xdd;
    P4DIR = 0xfd;

    P5OUT = 0xff;
    P5DIR = 0xff;

    P6OUT = 0x00;
    P6DIR = 0xff;

    P1IE = 0;
    P2IE = 0;






    MotePlatformC$uwait(1024 * 10);

    MotePlatformC$TOSH_FLASH_M25P_DP();
  }

  return SUCCESS;
}

# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
inline static error_t PlatformP$MoteInit$init(void ){
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
# 148 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP$startTimerB(void )
{

  Msp430ClockP$TBCTL = 0x0020 | (Msp430ClockP$TBCTL & ~(0x0020 | 0x0010));
}

#line 136
static inline void Msp430ClockP$startTimerA(void )
{

  Msp430ClockP$TA0CTL = 0x0020 | (Msp430ClockP$TA0CTL & ~(0x0020 | 0x0010));
}

#line 100
static inline void Msp430ClockP$Msp430ClockInit$defaultInitTimerB(void )
{
  TBR = 0;









  Msp430ClockP$TBCTL = 0x0100 | 0x0002;
}

#line 130
static inline void Msp430ClockP$Msp430ClockInit$default$initTimerB(void )
{
  Msp430ClockP$Msp430ClockInit$defaultInitTimerB();
}

# 32 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP$Msp430ClockInit$initTimerB(void ){
#line 32
  Msp430ClockP$Msp430ClockInit$default$initTimerB();
#line 32
}
#line 32
# 85 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP$Msp430ClockInit$defaultInitTimerA(void )
{
  TA0R = 0;









  Msp430ClockP$TA0CTL = 0x0200 | 0x0002;
}

#line 125
static inline void Msp430ClockP$Msp430ClockInit$default$initTimerA(void )
{
  Msp430ClockP$Msp430ClockInit$defaultInitTimerA();
}

# 31 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP$Msp430ClockInit$initTimerA(void ){
#line 31
  Msp430ClockP$Msp430ClockInit$default$initTimerA();
#line 31
}
#line 31
# 64 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP$Msp430ClockInit$defaultInitClocks(void )
{





  BCSCTL1 = 0x80 | (BCSCTL1 & ((0x04 | 0x02) | 0x01));







  BCSCTL2 = 0x04;


  Msp430ClockP$IE1 &= ~(1 << 1);
}

#line 120
static inline void Msp430ClockP$Msp430ClockInit$default$initClocks(void )
{
  Msp430ClockP$Msp430ClockInit$defaultInitClocks();
}

# 30 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP$Msp430ClockInit$initClocks(void ){
#line 30
  Msp430ClockP$Msp430ClockInit$default$initClocks();
#line 30
}
#line 30
# 166 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430ClockP.nc"
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




static inline void Msp430ClockP$busyCalibrateDco(void )
{

  int calib;
  int step;






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

#line 52
static inline void Msp430ClockP$Msp430ClockInit$defaultSetupDcoCalibrate(void )
{



  Msp430ClockP$TA0CTL = 0x0200 | 0x0020;
  Msp430ClockP$TBCTL = 0x0100 | 0x0020;
  BCSCTL1 = 0x80 | 0x04;
  BCSCTL2 = 0;
  TBCCTL0 = 0x4000;
}

#line 115
static inline void Msp430ClockP$Msp430ClockInit$default$setupDcoCalibrate(void )
{
  Msp430ClockP$Msp430ClockInit$defaultSetupDcoCalibrate();
}

# 29 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP$Msp430ClockInit$setupDcoCalibrate(void ){
#line 29
  Msp430ClockP$Msp430ClockInit$default$setupDcoCalibrate();
#line 29
}
#line 29
# 214 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline error_t Msp430ClockP$Init$init(void )
{

  Msp430ClockP$TA0CTL = 0x0004;
  Msp430ClockP$TA0IV = 0;
  Msp430ClockP$TBCTL = 0x0004;
  Msp430ClockP$TBIV = 0;
  /* atomic removed: atomic calls only */

  {
    Msp430ClockP$Msp430ClockInit$setupDcoCalibrate();
    Msp430ClockP$busyCalibrateDco();
    Msp430ClockP$Msp430ClockInit$initClocks();
    Msp430ClockP$Msp430ClockInit$initTimerA();
    Msp430ClockP$Msp430ClockInit$initTimerB();
    Msp430ClockP$startTimerA();
    Msp430ClockP$startTimerB();
  }

  return SUCCESS;
}

# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
inline static error_t PlatformP$MoteClockInit$init(void ){
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
# 10 "/opt/tinyos-2.1.0/tos/platforms/telosa/PlatformP.nc"
static inline error_t PlatformP$Init$init(void )
#line 10
{
  PlatformP$MoteClockInit$init();
  PlatformP$MoteInit$init();
  PlatformP$LedsInit$init();
  return SUCCESS;
}

# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
inline static error_t RealMainP$PlatformInit$init(void ){
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
# 33 "/opt/tinyos-2.1.0/tos/platforms/telosb/hardware.h"
static inline  void TOSH_CLR_SIMO0_PIN()
#line 33
{
#line 33
  static volatile uint8_t r __asm ("0x0019");

#line 33
  r &= ~(1 << 1);
}

# 54 "/opt/tinyos-2.1.0/tos/interfaces/Scheduler.nc"
inline static bool RealMainP$Scheduler$runNextTask(void ){
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
# 79 "/opt/tinyos-2.1.0/tos/system/ArbitratedReadStreamC.nc"
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$ReadStream$default$bufferDone(uint8_t client, error_t result, /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$val_t *buf, uint16_t count)
{
}

# 89 "/opt/tinyos-2.1.0/tos/interfaces/ReadStream.nc"
inline static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$ReadStream$bufferDone(uint8_t arg_0x40adb258, error_t result, /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$ReadStream$val_t * buf, uint16_t count){
#line 89
    /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$ReadStream$default$bufferDone(arg_0x40adb258, result, buf, count);
#line 89
}
#line 89
# 48 "/opt/tinyos-2.1.0/tos/system/ArbitratedReadStreamC.nc"
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$Service$bufferDone(uint8_t client, error_t result, /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$val_t *buf, uint16_t count)
{
  /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$ReadStream$bufferDone(client, result, buf, count);
}

# 89 "/opt/tinyos-2.1.0/tos/interfaces/ReadStream.nc"
inline static void AdcStreamP$ReadStream$bufferDone(uint8_t arg_0x40a869f0, error_t result, AdcStreamP$ReadStream$val_t * buf, uint16_t count){
#line 89
  /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$Service$bufferDone(arg_0x40a869f0, result, buf, count);
#line 89
}
#line 89
# 151 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline void AdcStreamP$bufferDone$runTask(void )
#line 151
{
  uint16_t *b;
#line 152
  uint16_t c;

#line 153
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      b = AdcStreamP$lastBuffer;
      c = AdcStreamP$lastCount;
      AdcStreamP$lastBuffer = (void *)0;
    }
#line 158
    __nesc_atomic_end(__nesc_atomic); }

  AdcStreamP$ReadStream$bufferDone(AdcStreamP$client, SUCCESS, b, c);
}

# 83 "/opt/tinyos-2.1.0/tos/system/ArbitratedReadStreamC.nc"
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$ReadStream$default$readDone(uint8_t client, error_t result, uint32_t actualPeriod)
{
}

# 102 "/opt/tinyos-2.1.0/tos/interfaces/ReadStream.nc"
inline static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$ReadStream$readDone(uint8_t arg_0x40adb258, error_t result, uint32_t usActualPeriod){
#line 102
    /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$ReadStream$default$readDone(arg_0x40adb258, result, usActualPeriod);
#line 102
}
#line 102
# 67 "/opt/tinyos-2.1.0/tos/system/ArbitratedReadStreamC.nc"
static inline error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$Resource$default$release(uint8_t client)
#line 67
{
#line 67
  return FAIL;
}

# 110 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
inline static error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$Resource$release(uint8_t arg_0x40ad7678){
#line 110
  unsigned char result;
#line 110

#line 110
  switch (arg_0x40ad7678) {
#line 110
    case /*AntiTheftAppC.ParLight.AdcReadStreamClientC*/AdcReadStreamClientC$0$RSCLIENT:
#line 110
      result = Msp430RefVoltArbiterImplP$ClientResource$release(/*AntiTheftAppC.ParLight.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$1$ID);
#line 110
      break;
#line 110
    default:
#line 110
      result = /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$Resource$default$release(arg_0x40ad7678);
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
# 53 "/opt/tinyos-2.1.0/tos/system/ArbitratedReadStreamC.nc"
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$Service$readDone(uint8_t client, error_t result, uint32_t actualPeriod)
{
  /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$Resource$release(client);
  /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$ReadStream$readDone(client, result, actualPeriod);
}

# 102 "/opt/tinyos-2.1.0/tos/interfaces/ReadStream.nc"
inline static void AdcStreamP$ReadStream$readDone(uint8_t arg_0x40a869f0, error_t result, uint32_t usActualPeriod){
#line 102
  /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$Service$readDone(arg_0x40a869f0, result, usActualPeriod);
#line 102
}
#line 102
# 130 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline void AdcStreamP$readStreamFail$runTask(void )
#line 130
{

  struct AdcStreamP$list_entry_t *entry;
  uint8_t c = AdcStreamP$client;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 135
    entry = AdcStreamP$bufferQueue[c];
#line 135
    __nesc_atomic_end(__nesc_atomic); }
  for (; entry; entry = entry->next) {
      uint16_t tmp_count __attribute((unused))  = entry->count;

#line 138
      AdcStreamP$ReadStream$bufferDone(c, FAIL, (uint16_t * )entry, entry->count);
    }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      AdcStreamP$bufferQueue[c] = (void *)0;
      AdcStreamP$bufferQueueEnd[c] = &AdcStreamP$bufferQueue[c];
    }
#line 145
    __nesc_atomic_end(__nesc_atomic); }

  AdcStreamP$client = AdcStreamP$NSTREAM;
  AdcStreamP$ReadStream$readDone(c, FAIL, 0);
}

# 170 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline error_t Msp430RefVoltArbiterImplP$AdcResource$default$release(uint8_t client)
#line 170
{
#line 170
  return FAIL;
}

# 110 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
inline static error_t Msp430RefVoltArbiterImplP$AdcResource$release(uint8_t arg_0x409692b8){
#line 110
  unsigned char result;
#line 110

#line 110
  switch (arg_0x409692b8) {
#line 110
    case /*AntiTheftAppC.ParLight.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID:
#line 110
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$release(/*AntiTheftAppC.ParLight.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID);
#line 110
      break;
#line 110
    case /*AntiTheftAppC.ParLight.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$1$ID:
#line 110
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$release(/*AntiTheftAppC.ParLight.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$1$ID);
#line 110
      break;
#line 110
    default:
#line 110
      result = Msp430RefVoltArbiterImplP$AdcResource$default$release(arg_0x409692b8);
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
# 56 "/opt/tinyos-2.1.0/tos/system/RoundRobinResourceQueueC.nc"
static inline bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$isEmpty(void )
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

# 43 "/opt/tinyos-2.1.0/tos/interfaces/ResourceQueue.nc"
inline static bool /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Queue$isEmpty(void ){
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
# 47 "/opt/tinyos-2.1.0/tos/system/RoundRobinResourceQueueC.nc"
static inline void /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$clearEntry(uint8_t id)
#line 47
{
  /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$resQ[id / 8] &= ~(1 << id % 8);
}

#line 69
static inline resource_client_id_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$dequeue(void )
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

# 60 "/opt/tinyos-2.1.0/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Queue$dequeue(void ){
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
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
inline static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask$postTask(void ){
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
# 172 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$unconfigure(uint8_t id)
#line 172
{
}

# 55 "/opt/tinyos-2.1.0/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$unconfigure(uint8_t arg_0x40902010){
#line 55
    /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$unconfigure(arg_0x40902010);
#line 55
}
#line 55
# 114 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline void AdcStreamP$readStreamDone$runTask(void )
#line 114
{
  uint8_t c = AdcStreamP$client;
  uint32_t actualPeriod = AdcStreamP$period;

#line 117
  if (AdcStreamP$periodModified) {
    actualPeriod = AdcStreamP$period - AdcStreamP$period % 1000;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      AdcStreamP$bufferQueue[c] = (void *)0;
      AdcStreamP$bufferQueueEnd[c] = &AdcStreamP$bufferQueue[c];
    }
#line 124
    __nesc_atomic_end(__nesc_atomic); }

  AdcStreamP$client = AdcStreamP$NSTREAM;
  AdcStreamP$ReadStream$readDone(c, SUCCESS, actualPeriod);
}

# 92 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type dt){
#line 92
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$startAt(t0, dt);
#line 92
}
#line 92
# 47 "/opt/tinyos-2.1.0/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$start(uint32_t t0, uint32_t dt, bool oneshot)
{
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_dt = dt;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_oneshot = oneshot;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$startAt(t0, dt);
}

#line 82
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$startOneShotAt(uint32_t t0, uint32_t dt)
{
#line 83
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$start(t0, dt, TRUE);
}

# 118 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$startOneShotAt(uint32_t t0, uint32_t dt){
#line 118
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$startOneShotAt(t0, dt);
#line 118
}
#line 118
# 54 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$disableEvents();
}

# 62 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$stop(void ){
#line 62
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$stop();
#line 62
}
#line 62
# 91 "/opt/tinyos-2.1.0/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$stop();
}

# 62 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$stop(void ){
#line 62
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$stop();
#line 62
}
#line 62
# 60 "/opt/tinyos-2.1.0/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$stop(void )
{
#line 61
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$stop();
}

# 67 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$stop(void ){
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$stop();
#line 67
}
#line 67
# 53 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
inline static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$get(void ){
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
# 75 "/opt/tinyos-2.1.0/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getNow(void )
{
  return /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$get();
}

# 98 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$getNow(void ){
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
# 85 "/opt/tinyos-2.1.0/tos/lib/timer/AlarmToTimerC.nc"
static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$getNow(void )
{
#line 86
  return /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$getNow();
}

# 125 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
inline static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow(void ){
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
# 89 "/opt/tinyos-2.1.0/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$runTask(void )
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
#line 124
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$startOneShotAt(now, min_remaining);
        }
    }
}

# 92 "/opt/tinyos-2.1.0/tos/interfaces/SplitControl.nc"
inline static void Msp430RefVoltGeneratorP$RefVolt_2_5V$startDone(error_t error){
#line 92
  Msp430RefVoltArbiterImplP$RefVolt_2_5V$startDone(error);
#line 92
}
#line 92
# 78 "/opt/tinyos-2.1.0/tos/interfaces/ReadStream.nc"
inline static error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$Service$read(uint8_t arg_0x40adad00, uint32_t usPeriod){
#line 78
  unsigned char result;
#line 78

#line 78
  result = AdcStreamP$ReadStream$read(arg_0x40adad00, usPeriod);
#line 78

#line 78
  return result;
#line 78
}
#line 78
# 59 "/opt/tinyos-2.1.0/tos/system/ArbitratedReadStreamC.nc"
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$Resource$granted(uint8_t client)
#line 59
{
  /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$Service$read(client, /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$period[client]);
}

# 160 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline void Msp430RefVoltArbiterImplP$ClientResource$default$granted(uint8_t client)
#line 160
{
}

# 92 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
inline static void Msp430RefVoltArbiterImplP$ClientResource$granted(uint8_t arg_0x409357d0){
#line 92
  switch (arg_0x409357d0) {
#line 92
    case /*AntiTheftAppC.ParLight.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID:
#line 92
      AdcP$ResourceRead$granted(/*AntiTheftAppC.ParLight.AdcReadClientC*/AdcReadClientC$0$CLIENT);
#line 92
      break;
#line 92
    case /*AntiTheftAppC.ParLight.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$1$ID:
#line 92
      /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC$0$Resource$granted(/*AntiTheftAppC.ParLight.AdcReadStreamClientC*/AdcReadStreamClientC$0$RSCLIENT);
#line 92
      break;
#line 92
    default:
#line 92
      Msp430RefVoltArbiterImplP$ClientResource$default$granted(arg_0x409357d0);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 98 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline void Msp430RefVoltArbiterImplP$RefVolt_1_5V$startDone(error_t error)
{
  if (Msp430RefVoltArbiterImplP$syncOwner != Msp430RefVoltArbiterImplP$NO_OWNER) {


      Msp430RefVoltArbiterImplP$ClientResource$granted(Msp430RefVoltArbiterImplP$syncOwner);
    }
}

# 92 "/opt/tinyos-2.1.0/tos/interfaces/SplitControl.nc"
inline static void Msp430RefVoltGeneratorP$RefVolt_1_5V$startDone(error_t error){
#line 92
  Msp430RefVoltArbiterImplP$RefVolt_1_5V$startDone(error);
#line 92
}
#line 92
# 220 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline void Msp430RefVoltGeneratorP$SwitchOnTimer$fired(void )
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

# 62 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
inline static void Msp430RefVoltGeneratorP$SwitchOffTimer$startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShot(1U, dt);
#line 62
}
#line 62
# 151 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline void Msp430RefVoltArbiterImplP$RefVolt_2_5V$stopDone(error_t error)
{
}

# 117 "/opt/tinyos-2.1.0/tos/interfaces/SplitControl.nc"
inline static void Msp430RefVoltGeneratorP$RefVolt_2_5V$stopDone(error_t error){
#line 117
  Msp430RefVoltArbiterImplP$RefVolt_2_5V$stopDone(error);
#line 117
}
#line 117
# 147 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline void Msp430RefVoltArbiterImplP$RefVolt_1_5V$stopDone(error_t error)
{
}

# 117 "/opt/tinyos-2.1.0/tos/interfaces/SplitControl.nc"
inline static void Msp430RefVoltGeneratorP$RefVolt_1_5V$stopDone(error_t error){
#line 117
  Msp430RefVoltArbiterImplP$RefVolt_1_5V$stopDone(error);
#line 117
}
#line 117
# 244 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline void Msp430RefVoltGeneratorP$SwitchOffTimer$fired(void )
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

# 161 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline error_t Msp430RefVoltArbiterImplP$AdcResource$default$request(uint8_t client)
{
  return FAIL;
}

# 78 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
inline static error_t Msp430RefVoltArbiterImplP$AdcResource$request(uint8_t arg_0x409692b8){
#line 78
  unsigned char result;
#line 78

#line 78
  switch (arg_0x409692b8) {
#line 78
    case /*AntiTheftAppC.ParLight.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID:
#line 78
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$request(/*AntiTheftAppC.ParLight.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID);
#line 78
      break;
#line 78
    case /*AntiTheftAppC.ParLight.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$1$ID:
#line 78
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$request(/*AntiTheftAppC.ParLight.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$1$ID);
#line 78
      break;
#line 78
    default:
#line 78
      result = Msp430RefVoltArbiterImplP$AdcResource$default$request(arg_0x409692b8);
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
# 53 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline error_t Msp430RefVoltArbiterImplP$ClientResource$request(uint8_t client)
{
  return Msp430RefVoltArbiterImplP$AdcResource$request(client);
}

# 168 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcP.nc"
static inline error_t AdcP$ResourceRead$default$request(uint8_t client)
#line 168
{
#line 168
  return FAIL;
}

# 78 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
inline static error_t AdcP$ResourceRead$request(uint8_t arg_0x40817890){
#line 78
  unsigned char result;
#line 78

#line 78
  switch (arg_0x40817890) {
#line 78
    case /*AntiTheftAppC.ParLight.AdcReadClientC*/AdcReadClientC$0$CLIENT:
#line 78
      result = Msp430RefVoltArbiterImplP$ClientResource$request(/*AntiTheftAppC.ParLight.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID);
#line 78
      break;
#line 78
    default:
#line 78
      result = AdcP$ResourceRead$default$request(arg_0x40817890);
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
# 75 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcP.nc"
static inline error_t AdcP$Read$read(uint8_t client)
{
  return AdcP$ResourceRead$request(client);
}

# 55 "/opt/tinyos-2.1.0/tos/interfaces/Read.nc"
inline static error_t DarkC$Light$read(void ){
#line 55
  unsigned char result;
#line 55

#line 55
  result = AdcP$Read$read(/*AntiTheftAppC.ParLight.AdcReadClientC*/AdcReadClientC$0$CLIENT);
#line 55

#line 55
  return result;
#line 55
}
#line 55
# 28 "DarkC.nc"
static inline void DarkC$TheftTimer$fired(void )
{
  DarkC$Light$read();
}

# 193 "/opt/tinyos-2.1.0/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$default$fired(uint8_t num)
{
}

# 72 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$fired(uint8_t arg_0x40a2b9f0){
#line 72
  switch (arg_0x40a2b9f0) {
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
      DarkC$TheftTimer$fired();
#line 72
      break;
#line 72
    default:
#line 72
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$default$fired(arg_0x40a2b9f0);
#line 72
      break;
#line 72
    }
#line 72
}
#line 72
# 166 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceRequested$default$requested(uint8_t id)
#line 166
{
}

# 43 "/opt/tinyos-2.1.0/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceRequested$requested(uint8_t arg_0x40903318){
#line 43
    /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceRequested$default$requested(arg_0x40903318);
#line 43
}
#line 43
# 87 "/opt/tinyos-2.1.0/tos/system/RoundRobinResourceQueueC.nc"
static inline error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$enqueue(resource_client_id_t id)
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

# 69 "/opt/tinyos-2.1.0/tos/interfaces/ResourceQueue.nc"
inline static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Queue$enqueue(resource_client_id_t id){
#line 69
  unsigned char result;
#line 69

#line 69
  result = /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$enqueue(id);
#line 69

#line 69
  return result;
#line 69
}
#line 69
# 121 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline bool HplAdc12P$HplAdc12$isBusy(void )
#line 121
{
#line 121
  return HplAdc12P$ADC12CTL1 & 0x0001;
}

# 118 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/HplAdc12.nc"
inline static bool Msp430RefVoltGeneratorP$HplAdc12$isBusy(void ){
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
# 69 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline adc12ctl0_t HplAdc12P$HplAdc12$getCtl0(void )
#line 69
{
  return * (adc12ctl0_t *)&HplAdc12P$ADC12CTL0;
}

# 63 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/HplAdc12.nc"
inline static adc12ctl0_t Msp430RefVoltGeneratorP$HplAdc12$getCtl0(void ){
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
# 61 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline void HplAdc12P$HplAdc12$setCtl0(adc12ctl0_t control0)
#line 61
{
  HplAdc12P$ADC12CTL0 = * (uint16_t *)&control0;
}

# 51 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/HplAdc12.nc"
inline static void Msp430RefVoltGeneratorP$HplAdc12$setCtl0(adc12ctl0_t control0){
#line 51
  HplAdc12P$HplAdc12$setCtl0(control0);
#line 51
}
#line 51
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$postTask(void ){
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
# 53 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
inline static /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$Counter$size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$Counter$get(void ){
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
# 75 "/opt/tinyos-2.1.0/tos/lib/timer/TransformAlarmC.nc"
static inline /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$to_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$Alarm$getNow(void )
{
  return /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$Counter$get();
}

# 98 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
inline static AdcStreamP$Alarm$size_type AdcStreamP$Alarm$getNow(void ){
#line 98
  unsigned long result;
#line 98

#line 98
  result = /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$Alarm$getNow();
#line 98

#line 98
  return result;
#line 98
}
#line 98
# 319 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline error_t AdcStreamP$SingleChannel$default$configureSingle(uint8_t c, 
const msp430adc12_channel_config_t *config)
#line 320
{
#line 320
  return FAIL;
}

# 84 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static error_t AdcStreamP$SingleChannel$configureSingle(uint8_t arg_0x40aa8598, const msp430adc12_channel_config_t * config){
#line 84
  unsigned char result;
#line 84

#line 84
  switch (arg_0x40aa8598) {
#line 84
    case /*AntiTheftAppC.ParLight.AdcReadStreamClientC*/AdcReadStreamClientC$0$RSCLIENT:
#line 84
      result = Msp430Adc12ImplP$SingleChannel$configureSingle(/*AntiTheftAppC.ParLight.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$1$ID, config);
#line 84
      break;
#line 84
    default:
#line 84
      result = AdcStreamP$SingleChannel$default$configureSingle(arg_0x40aa8598, config);
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
# 61 "/opt/tinyos-2.1.0/tos/platforms/telosa/chips/s1087/HamamatsuS1087ParP.nc"
static inline const msp430adc12_channel_config_t *HamamatsuS1087ParP$AdcConfigure$getConfiguration(void )
#line 61
{
  return &HamamatsuS1087ParP$config;
}

# 305 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline const msp430adc12_channel_config_t *AdcStreamP$AdcConfigure$default$getConfiguration(uint8_t c)
{
  return &AdcStreamP$defaultConfig;
}

# 58 "/opt/tinyos-2.1.0/tos/interfaces/AdcConfigure.nc"
inline static AdcStreamP$AdcConfigure$adc_config_t AdcStreamP$AdcConfigure$getConfiguration(uint8_t arg_0x40aa7428){
#line 58
  struct __nesc_unnamed4267 const *result;
#line 58

#line 58
  switch (arg_0x40aa7428) {
#line 58
    case /*AntiTheftAppC.ParLight.AdcReadStreamClientC*/AdcReadStreamClientC$0$RSCLIENT:
#line 58
      result = HamamatsuS1087ParP$AdcConfigure$getConfiguration();
#line 58
      break;
#line 58
    default:
#line 58
      result = AdcStreamP$AdcConfigure$default$getConfiguration(arg_0x40aa7428);
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
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
inline static error_t AdcStreamP$readStreamDone$postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(AdcStreamP$readStreamDone);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 136 "/opt/tinyos-2.1.0/tos/lib/timer/TransformAlarmC.nc"
static inline void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$Alarm$startAt(/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$to_size_type t0, /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$to_size_type dt)
{
  /* atomic removed: atomic calls only */
  {
    /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$m_t0 = t0;
    /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$m_dt = dt;
    /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$set_alarm();
  }
}

# 92 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
inline static void AdcStreamP$Alarm$startAt(AdcStreamP$Alarm$size_type t0, AdcStreamP$Alarm$size_type dt){
#line 92
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$Alarm$startAt(t0, dt);
#line 92
}
#line 92
# 163 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline void AdcStreamP$nextAlarm(void )
#line 163
{
  AdcStreamP$Alarm$startAt(AdcStreamP$now, AdcStreamP$period);
  AdcStreamP$now += AdcStreamP$period;
}

# 144 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$setEvent(uint16_t x)
{
  * (volatile uint16_t * )372U = x;
}

# 30 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void Msp430Adc12ImplP$CompareA1$setEvent(uint16_t time){
#line 30
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$setEvent(time);
#line 30
}
#line 30
# 144 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$setEvent(uint16_t x)
{
  * (volatile uint16_t * )370U = x;
}

# 30 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void Msp430Adc12ImplP$CompareA0$setEvent(uint16_t time){
#line 30
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$setEvent(time);
#line 30
}
#line 30
# 46 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$CC2int(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t x)
#line 46
{
#line 46
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$__nesc_unnamed4310 {
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
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Control$setControl(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t x)
{
  * (volatile uint16_t * )354U = /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$CC2int(x);
}

# 35 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void Msp430Adc12ImplP$ControlA0$setControl(msp430_compare_control_t control){
#line 35
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Control$setControl(control);
#line 35
}
#line 35
# 110 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setInputDivider(uint16_t inputDivider)
{
  * (volatile uint16_t * )352U = (* (volatile uint16_t * )352U & ~(0x0040 | 0x0080)) | ((inputDivider << 6) & (0x0040 | 0x0080));
}

# 45 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void Msp430Adc12ImplP$TimerA$setInputDivider(uint16_t inputDivider){
#line 45
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setInputDivider(inputDivider);
#line 45
}
#line 45
# 105 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setClockSource(uint16_t clockSource)
{
  * (volatile uint16_t * )352U = (* (volatile uint16_t * )352U & ~(256U | 512U)) | ((clockSource << 8) & (256U | 512U));
}

# 44 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void Msp430Adc12ImplP$TimerA$setClockSource(uint16_t clockSource){
#line 44
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setClockSource(clockSource);
#line 44
}
#line 44
# 100 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$disableEvents(void )
{
  * (volatile uint16_t * )352U &= ~2U;
}

# 43 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void Msp430Adc12ImplP$TimerA$disableEvents(void ){
#line 43
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$disableEvents();
#line 43
}
#line 43
# 90 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$clear(void )
{
  * (volatile uint16_t * )352U |= 4U;
}

# 41 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void Msp430Adc12ImplP$TimerA$clear(void ){
#line 41
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$clear();
#line 41
}
#line 41
# 99 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
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

# 95 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline void HplAdc12P$HplAdc12$setIEFlags(uint16_t mask)
#line 95
{
#line 95
  HplAdc12P$ADC12IE = mask;
}

# 95 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/HplAdc12.nc"
inline static void Msp430Adc12ImplP$HplAdc12$setIEFlags(uint16_t mask){
#line 95
  HplAdc12P$HplAdc12$setIEFlags(mask);
#line 95
}
#line 95
# 77 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline void HplAdc12P$HplAdc12$setMCtl(uint8_t i, adc12memctl_t memControl)
#line 77
{
  uint8_t *memCtlPtr = (uint8_t *)(char *)0x0080;

#line 79
  memCtlPtr += i;
  *memCtlPtr = * (uint8_t *)&memControl;
}

# 75 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/HplAdc12.nc"
inline static void Msp430Adc12ImplP$HplAdc12$setMCtl(uint8_t idx, adc12memctl_t memControl){
#line 75
  HplAdc12P$HplAdc12$setMCtl(idx, memControl);
#line 75
}
#line 75
# 65 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline void HplAdc12P$HplAdc12$setCtl1(adc12ctl1_t control1)
#line 65
{
  HplAdc12P$ADC12CTL1 = * (uint16_t *)&control1;
}

# 57 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/HplAdc12.nc"
inline static void Msp430Adc12ImplP$HplAdc12$setCtl1(adc12ctl1_t control1){
#line 57
  HplAdc12P$HplAdc12$setCtl1(control1);
#line 57
}
#line 57
#line 51
inline static void Msp430Adc12ImplP$HplAdc12$setCtl0(adc12ctl0_t control0){
#line 51
  HplAdc12P$HplAdc12$setCtl0(control0);
#line 51
}
#line 51
#line 63
inline static adc12ctl0_t Msp430Adc12ImplP$HplAdc12$getCtl0(void ){
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
# 88 "/opt/tinyos-2.1.0/tos/interfaces/ArbiterInfo.nc"
inline static uint8_t Msp430Adc12ImplP$ADCArbiterInfo$userId(void ){
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
# 263 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline error_t Msp430Adc12ImplP$SingleChannel$configureMultiple(uint8_t id, 
const msp430adc12_channel_config_t *config, 
uint16_t *buf, uint16_t length, uint16_t jiffies)
{
  error_t result = ERESERVE;

  if ((((!config || !buf) || !length) || jiffies == 1) || jiffies == 2) {
    return EINVAL;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 272
    {
      if (Msp430Adc12ImplP$state & Msp430Adc12ImplP$ADC_BUSY) 
        {
          unsigned char __nesc_temp = 
#line 274
          EBUSY;

          {
#line 274
            __nesc_atomic_end(__nesc_atomic); 
#line 274
            return __nesc_temp;
          }
        }
#line 275
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
#line 291
          uint16_t mask = 1;
          adc12ctl0_t ctl0 = Msp430Adc12ImplP$HplAdc12$getCtl0();

#line 293
          ctl0.msc = jiffies == 0 ? 1 : 0;
          ctl0.sht0 = config->sht;
          ctl0.sht1 = config->sht;

          Msp430Adc12ImplP$state = Msp430Adc12ImplP$MULTIPLE_DATA;
          Msp430Adc12ImplP$resultBufferStart = (void *)0;
          Msp430Adc12ImplP$resultBufferLength = length;
          Msp430Adc12ImplP$resultBufferStart = buf;
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
#line 316
    __nesc_atomic_end(__nesc_atomic); }
  return result;
}

# 309 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline error_t AdcStreamP$SingleChannel$default$configureMultiple(uint8_t c, 
const msp430adc12_channel_config_t *config, uint16_t b[], 
uint16_t numSamples, uint16_t jiffies)
{
  return FAIL;
}

# 138 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static error_t AdcStreamP$SingleChannel$configureMultiple(uint8_t arg_0x40aa8598, const msp430adc12_channel_config_t * config, uint16_t * buffer, uint16_t numSamples, uint16_t jiffies){
#line 138
  unsigned char result;
#line 138

#line 138
  switch (arg_0x40aa8598) {
#line 138
    case /*AntiTheftAppC.ParLight.AdcReadStreamClientC*/AdcReadStreamClientC$0$RSCLIENT:
#line 138
      result = Msp430Adc12ImplP$SingleChannel$configureMultiple(/*AntiTheftAppC.ParLight.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$1$ID, config, buffer, numSamples, jiffies);
#line 138
      break;
#line 138
    default:
#line 138
      result = AdcStreamP$SingleChannel$default$configureMultiple(arg_0x40aa8598, config, buffer, numSamples, jiffies);
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
# 191 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcP.nc"
static inline error_t AdcP$SingleChannel$default$configureSingle(uint8_t client, 
const msp430adc12_channel_config_t *config)
#line 192
{
#line 192
  return FAIL;
}

# 84 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static error_t AdcP$SingleChannel$configureSingle(uint8_t arg_0x40823e40, const msp430adc12_channel_config_t * config){
#line 84
  unsigned char result;
#line 84

#line 84
  switch (arg_0x40823e40) {
#line 84
    case /*AntiTheftAppC.ParLight.AdcReadClientC*/AdcReadClientC$0$CLIENT:
#line 84
      result = Msp430Adc12ImplP$SingleChannel$configureSingle(/*AntiTheftAppC.ParLight.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID, config);
#line 84
      break;
#line 84
    default:
#line 84
      result = AdcP$SingleChannel$default$configureSingle(arg_0x40823e40, config);
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
# 186 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcP.nc"
static inline const msp430adc12_channel_config_t *
AdcP$Config$default$getConfiguration(uint8_t client)
{
  return &AdcP$defaultConfig;
}

# 58 "/opt/tinyos-2.1.0/tos/interfaces/AdcConfigure.nc"
inline static AdcP$Config$adc_config_t AdcP$Config$getConfiguration(uint8_t arg_0x40812068){
#line 58
  struct __nesc_unnamed4267 const *result;
#line 58

#line 58
  switch (arg_0x40812068) {
#line 58
    case /*AntiTheftAppC.ParLight.AdcReadClientC*/AdcReadClientC$0$CLIENT:
#line 58
      result = HamamatsuS1087ParP$AdcConfigure$getConfiguration();
#line 58
      break;
#line 58
    default:
#line 58
      result = AdcP$Config$default$getConfiguration(arg_0x40812068);
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
# 65 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcP.nc"
static inline error_t AdcP$configure(uint8_t client)
{
  error_t result = EINVAL;
  const msp430adc12_channel_config_t * config;

#line 69
  config = AdcP$Config$getConfiguration(client);
  if (config->inch != INPUT_CHANNEL_NONE) {
    result = AdcP$SingleChannel$configureSingle(client, config);
    }
#line 72
  return result;
}

#line 180
static inline error_t AdcP$SingleChannel$default$getData(uint8_t client)
{
  return EINVAL;
}

# 189 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static error_t AdcP$SingleChannel$getData(uint8_t arg_0x40823e40){
#line 189
  unsigned char result;
#line 189

#line 189
  switch (arg_0x40823e40) {
#line 189
    case /*AntiTheftAppC.ParLight.AdcReadClientC*/AdcReadClientC$0$CLIENT:
#line 189
      result = Msp430Adc12ImplP$SingleChannel$getData(/*AntiTheftAppC.ParLight.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID);
#line 189
      break;
#line 189
    default:
#line 189
      result = AdcP$SingleChannel$default$getData(arg_0x40823e40);
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
# 46 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$clr(void )
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t * )49U &= ~(0x01 << 6);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}

# 39 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$clr();
#line 39
}
#line 39
# 38 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$clr(void )
#line 38
{
#line 38
  /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$clr();
}

# 30 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led2$clr(void ){
#line 30
  /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$clr();
#line 30
}
#line 30
# 93 "/opt/tinyos-2.1.0/tos/system/LedsP.nc"
static inline void LedsP$Leds$led2On(void )
#line 93
{
  LedsP$Led2$clr();
  ;
#line 95
  ;
}

# 78 "/opt/tinyos-2.1.0/tos/interfaces/Leds.nc"
inline static void DarkC$Leds$led2On(void ){
#line 78
  LedsP$Leds$led2On();
#line 78
}
#line 78
# 98 "/opt/tinyos-2.1.0/tos/system/LedsP.nc"
static inline void LedsP$Leds$led2Off(void )
#line 98
{
  LedsP$Led2$set();
  ;
#line 100
  ;
}

# 83 "/opt/tinyos-2.1.0/tos/interfaces/Leds.nc"
inline static void DarkC$Leds$led2Off(void ){
#line 83
  LedsP$Leds$led2Off();
#line 83
}
#line 83
# 128 "/opt/tinyos-2.1.0/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$fired(void )
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$fireTimers(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow());
}

# 72 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$fired(void ){
#line 72
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$fired();
#line 72
}
#line 72
# 80 "/opt/tinyos-2.1.0/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getAlarm(void )
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

# 105 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$getAlarm(void ){
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
# 63 "/opt/tinyos-2.1.0/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$runTask(void )
{
  if (/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_oneshot == FALSE) {
    /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$start(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$getAlarm(), /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_dt, FALSE);
    }
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$fired();
}

# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
inline static error_t Msp430RefVoltArbiterImplP$switchOff$postTask(void ){
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
# 153 "/opt/tinyos-2.1.0/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$stop(uint8_t num)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_timers[num].isrunning = FALSE;
}

# 67 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
inline static void Msp430RefVoltGeneratorP$SwitchOnTimer$stop(void ){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$stop(0U);
#line 67
}
#line 67
# 127 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline error_t Msp430RefVoltGeneratorP$RefVolt_1_5V$stop(void )
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

# 109 "/opt/tinyos-2.1.0/tos/interfaces/SplitControl.nc"
inline static error_t Msp430RefVoltArbiterImplP$RefVolt_1_5V$stop(void ){
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
# 136 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline void Msp430RefVoltArbiterImplP$switchOff$runTask(void )
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

# 164 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$default$granted(uint8_t id)
#line 164
{
}

# 92 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
inline static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$granted(uint8_t arg_0x409048e0){
#line 92
  switch (arg_0x409048e0) {
#line 92
    case /*AntiTheftAppC.ParLight.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID:
#line 92
      Msp430RefVoltArbiterImplP$AdcResource$granted(/*AntiTheftAppC.ParLight.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID);
#line 92
      break;
#line 92
    case /*AntiTheftAppC.ParLight.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$1$ID:
#line 92
      Msp430RefVoltArbiterImplP$AdcResource$granted(/*AntiTheftAppC.ParLight.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$1$ID);
#line 92
      break;
#line 92
    default:
#line 92
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$default$granted(arg_0x409048e0);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 170 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$configure(uint8_t id)
#line 170
{
}

# 49 "/opt/tinyos-2.1.0/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$configure(uint8_t arg_0x40902010){
#line 49
    /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$configure(arg_0x40902010);
#line 49
}
#line 49
# 154 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask$runTask(void )
#line 154
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 155
    {
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$resId = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$reqResId;
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$state = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$RES_BUSY;
    }
#line 158
    __nesc_atomic_end(__nesc_atomic); }
  /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$configure(/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$resId);
  /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$granted(/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$resId);
}

# 58 "/opt/tinyos-2.1.0/tos/interfaces/AdcConfigure.nc"
inline static /*AntiTheftAppC.ParLight.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$0$ConfUp$adc_config_t /*AntiTheftAppC.ParLight.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$0$ConfUp$getConfiguration(void ){
#line 58
  struct __nesc_unnamed4267 const *result;
#line 58

#line 58
  result = HamamatsuS1087ParP$AdcConfigure$getConfiguration();
#line 58

#line 58
  return result;
#line 58
}
#line 58
# 47 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline const msp430adc12_channel_config_t */*AntiTheftAppC.ParLight.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$0$ConfSub$getConfiguration(void )
{
  return /*AntiTheftAppC.ParLight.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$0$ConfUp$getConfiguration();
}

# 58 "/opt/tinyos-2.1.0/tos/interfaces/AdcConfigure.nc"
inline static /*AntiTheftAppC.ParLight.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$1$ConfUp$adc_config_t /*AntiTheftAppC.ParLight.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$1$ConfUp$getConfiguration(void ){
#line 58
  struct __nesc_unnamed4267 const *result;
#line 58

#line 58
  result = HamamatsuS1087ParP$AdcConfigure$getConfiguration();
#line 58

#line 58
  return result;
#line 58
}
#line 58
# 47 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline const msp430adc12_channel_config_t */*AntiTheftAppC.ParLight.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$1$ConfSub$getConfiguration(void )
{
  return /*AntiTheftAppC.ParLight.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$1$ConfUp$getConfiguration();
}

# 172 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline const msp430adc12_channel_config_t *
Msp430RefVoltArbiterImplP$Config$default$getConfiguration(uint8_t client)
{
  return &Msp430RefVoltArbiterImplP$defaultConfig;
}

# 58 "/opt/tinyos-2.1.0/tos/interfaces/AdcConfigure.nc"
inline static Msp430RefVoltArbiterImplP$Config$adc_config_t Msp430RefVoltArbiterImplP$Config$getConfiguration(uint8_t arg_0x40968d10){
#line 58
  struct __nesc_unnamed4267 const *result;
#line 58

#line 58
  switch (arg_0x40968d10) {
#line 58
    case /*AntiTheftAppC.ParLight.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID:
#line 58
      result = /*AntiTheftAppC.ParLight.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$0$ConfSub$getConfiguration();
#line 58
      break;
#line 58
    case /*AntiTheftAppC.ParLight.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$1$ID:
#line 58
      result = /*AntiTheftAppC.ParLight.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC$1$ConfSub$getConfiguration();
#line 58
      break;
#line 58
    default:
#line 58
      result = Msp430RefVoltArbiterImplP$Config$default$getConfiguration(arg_0x40968d10);
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
# 67 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
inline static void Msp430RefVoltGeneratorP$SwitchOffTimer$stop(void ){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$stop(1U);
#line 67
}
#line 67
#line 62
inline static void Msp430RefVoltGeneratorP$SwitchOnTimer$startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShot(0U, dt);
#line 62
}
#line 62
# 94 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline error_t Msp430RefVoltGeneratorP$RefVolt_1_5V$start(void )
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

# 83 "/opt/tinyos-2.1.0/tos/interfaces/SplitControl.nc"
inline static error_t Msp430RefVoltArbiterImplP$RefVolt_1_5V$start(void ){
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
# 157 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline error_t Msp430RefVoltGeneratorP$RefVolt_2_5V$start(void )
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

# 83 "/opt/tinyos-2.1.0/tos/interfaces/SplitControl.nc"
inline static error_t Msp430RefVoltArbiterImplP$RefVolt_2_5V$start(void ){
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
# 172 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcP.nc"
static inline void AdcP$Read$default$readDone(uint8_t client, error_t result, uint16_t val)
#line 172
{
}

# 63 "/opt/tinyos-2.1.0/tos/interfaces/Read.nc"
inline static void AdcP$Read$readDone(uint8_t arg_0x40801668, error_t result, AdcP$Read$val_t val){
#line 63
  switch (arg_0x40801668) {
#line 63
    case /*AntiTheftAppC.ParLight.AdcReadClientC*/AdcReadClientC$0$CLIENT:
#line 63
      DarkC$Light$readDone(result, val);
#line 63
      break;
#line 63
    default:
#line 63
      AdcP$Read$default$readDone(arg_0x40801668, result, val);
#line 63
      break;
#line 63
    }
#line 63
}
#line 63
# 170 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcP.nc"
static inline error_t AdcP$ResourceRead$default$release(uint8_t client)
#line 170
{
#line 170
  return FAIL;
}

# 110 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
inline static error_t AdcP$ResourceRead$release(uint8_t arg_0x40817890){
#line 110
  unsigned char result;
#line 110

#line 110
  switch (arg_0x40817890) {
#line 110
    case /*AntiTheftAppC.ParLight.AdcReadClientC*/AdcReadClientC$0$CLIENT:
#line 110
      result = Msp430RefVoltArbiterImplP$ClientResource$release(/*AntiTheftAppC.ParLight.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID);
#line 110
      break;
#line 110
    default:
#line 110
      result = AdcP$ResourceRead$default$release(arg_0x40817890);
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
# 136 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcP.nc"
static inline void AdcP$readDone$runTask(void )
{
  AdcP$ResourceRead$release(AdcP$owner);
  AdcP$Read$readDone(AdcP$owner, SUCCESS, AdcP$value);
}

# 58 "/opt/tinyos-2.1.0/tos/types/TinyError.h"
static inline  error_t ecombine(error_t r1, error_t r2)




{
  return r1 == r2 ? r1 : FAIL;
}

# 123 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/HplAdc12.nc"
inline static void Msp430Adc12ImplP$HplAdc12$stopConversion(void ){
#line 123
  HplAdc12P$HplAdc12$stopConversion();
#line 123
}
#line 123
# 88 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline error_t Msp430Adc12ImplP$Init$init(void )
{
  adc12ctl0_t ctl0;

#line 91
  Msp430Adc12ImplP$HplAdc12$stopConversion();
  ctl0 = Msp430Adc12ImplP$HplAdc12$getCtl0();
  ctl0.adc12tovie = 1;
  ctl0.adc12ovie = 1;
  Msp430Adc12ImplP$HplAdc12$setCtl0(ctl0);
  return SUCCESS;
}

# 51 "/opt/tinyos-2.1.0/tos/system/RoundRobinResourceQueueC.nc"
static inline error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$Init$init(void )
#line 51
{
  memset(/*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$resQ, 0, sizeof /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$resQ);
  return SUCCESS;
}

# 46 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t x)
#line 46
{
#line 46
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$__nesc_unnamed4311 {
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

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$compareControl(void )
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
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$setControlAsCompare(void )
{
  * (volatile uint16_t * )386U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$compareControl();
}

# 36 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$setControlAsCompare(void ){
#line 36
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$setControlAsCompare();
#line 36
}
#line 36
# 42 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Init$init(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$setControlAsCompare();
  return SUCCESS;
}

# 83 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline error_t AdcStreamP$Init$init(void )
#line 83
{
  uint8_t i;

  for (i = 0; i != AdcStreamP$NSTREAM; i++) 
    AdcStreamP$bufferQueueEnd[i] = &AdcStreamP$bufferQueue[i];

  return SUCCESS;
}

# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
inline static error_t RealMainP$SoftwareInit$init(void ){
#line 51
  unsigned char result;
#line 51

#line 51
  result = AdcStreamP$Init$init();
#line 51
  result = ecombine(result, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Init$init());
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
# 143 "/opt/tinyos-2.1.0/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startPeriodic(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow(), dt, FALSE);
}

# 53 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
inline static void DarkC$TheftTimer$startPeriodic(uint32_t dt){
#line 53
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startPeriodic(2U, dt);
#line 53
}
#line 53
# 23 "DarkC.nc"
static inline void DarkC$Boot$booted(void )
{
  DarkC$TheftTimer$startPeriodic(DarkC$DARK_INTERVAL);
}

# 49 "/opt/tinyos-2.1.0/tos/interfaces/Boot.nc"
inline static void RealMainP$Boot$booted(void ){
#line 49
  DarkC$Boot$booted();
#line 49
}
#line 49
# 206 "/opt/tinyos-2.1.0/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_disable_interrupt(void )
{
   __asm volatile ("dint");
   __asm volatile ("nop");}

# 126 "/opt/tinyos-2.1.0/tos/chips/msp430/McuSleepC.nc"
static inline mcu_power_t McuSleepC$McuPowerOverride$default$lowestState(void )
#line 126
{
  return MSP430_POWER_LPM4;
}

# 54 "/opt/tinyos-2.1.0/tos/interfaces/McuPowerOverride.nc"
inline static mcu_power_t McuSleepC$McuPowerOverride$lowestState(void ){
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
# 66 "/opt/tinyos-2.1.0/tos/chips/msp430/McuSleepC.nc"
static inline mcu_power_t McuSleepC$getPowerState(void )
#line 66
{
  mcu_power_t pState = MSP430_POWER_LPM3;









  if ((((((
#line 69
  TA0CCTL0 & 0x0010 || 
  TA0CCTL1 & 0x0010) || 
  TA0CCTL2 & 0x0010) && (
  TA0CTL & (3 << 8)) == 2 << 8) || (
  ME1 & ((1 << 7) | (1 << 6)) && U0TCTL & 0x20)) || (
  ME2 & ((1 << 5) | (1 << 4)) && U1TCTL & 0x20))


   || (U0CTLnr & 0x01 && I2CTCTLnr & 0x20 && 
  I2CDCTLnr & 0x20 && U0CTLnr & 0x04 && U0CTLnr & 0x20)) {


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

# 194 "/opt/tinyos-2.1.0/tos/chips/msp430/msp430hardware.h"
static inline  mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)
#line 194
{
  return m1 < m2 ? m1 : m2;
}

# 104 "/opt/tinyos-2.1.0/tos/chips/msp430/McuSleepC.nc"
static inline void McuSleepC$computePowerState(void )
#line 104
{
  McuSleepC$powerState = mcombine(McuSleepC$getPowerState(), 
  McuSleepC$McuPowerOverride$lowestState());
}

static inline void McuSleepC$McuSleep$sleep(void )
#line 109
{
  uint16_t temp;

#line 111
  if (McuSleepC$dirty) {
      McuSleepC$computePowerState();
    }

  temp = McuSleepC$msp430PowerBits[McuSleepC$powerState] | 0x0008;
   __asm volatile ("bis  %0, r2" :  : "m"(temp));

   __asm volatile ("" :  :  : "memory");
  __nesc_disable_interrupt();
}

# 59 "/opt/tinyos-2.1.0/tos/interfaces/McuSleep.nc"
inline static void SchedulerBasicP$McuSleep$sleep(void ){
#line 59
  McuSleepC$McuSleep$sleep();
#line 59
}
#line 59
# 67 "/opt/tinyos-2.1.0/tos/system/SchedulerBasicP.nc"
static __inline uint8_t SchedulerBasicP$popTask(void )
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
static inline void SchedulerBasicP$Scheduler$taskLoop(void )
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

# 61 "/opt/tinyos-2.1.0/tos/interfaces/Scheduler.nc"
inline static void RealMainP$Scheduler$taskLoop(void ){
#line 61
  SchedulerBasicP$Scheduler$taskLoop();
#line 61
}
#line 61
# 161 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcP.nc"
static inline uint16_t *AdcP$SingleChannel$multipleDataReady(uint8_t client, 
uint16_t *buf, uint16_t numSamples)
{

  return 0;
}

# 625 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline uint16_t *Msp430Adc12ImplP$SingleChannel$default$multipleDataReady(uint8_t id, 
uint16_t *buf, uint16_t numSamples)
{
  return 0;
}

# 227 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static uint16_t * Msp430Adc12ImplP$SingleChannel$multipleDataReady(uint8_t arg_0x408593f0, uint16_t * buffer, uint16_t numSamples){
#line 227
  unsigned int *result;
#line 227

#line 227
  switch (arg_0x408593f0) {
#line 227
    case /*AntiTheftAppC.ParLight.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID:
#line 227
      result = AdcP$SingleChannel$multipleDataReady(/*AntiTheftAppC.ParLight.AdcReadClientC*/AdcReadClientC$0$CLIENT, buffer, numSamples);
#line 227
      break;
#line 227
    case /*AntiTheftAppC.ParLight.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$1$ID:
#line 227
      result = AdcStreamP$SingleChannel$multipleDataReady(/*AntiTheftAppC.ParLight.AdcReadStreamClientC*/AdcReadStreamClientC$0$RSCLIENT, buffer, numSamples);
#line 227
      break;
#line 227
    default:
#line 227
      result = Msp430Adc12ImplP$SingleChannel$default$multipleDataReady(arg_0x408593f0, buffer, numSamples);
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
# 91 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline uint16_t HplAdc12P$HplAdc12$getMem(uint8_t i)
#line 91
{
  return *((uint16_t *)(int *)0x0140 + i);
}

# 89 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/HplAdc12.nc"
inline static uint16_t Msp430Adc12ImplP$HplAdc12$getMem(uint8_t idx){
#line 89
  unsigned int result;
#line 89

#line 89
  result = HplAdc12P$HplAdc12$getMem(idx);
#line 89

#line 89
  return result;
#line 89
}
#line 89
#line 82
inline static adc12memctl_t Msp430Adc12ImplP$HplAdc12$getMCtl(uint8_t idx){
#line 82
  struct __nesc_unnamed4268 result;
#line 82

#line 82
  result = HplAdc12P$HplAdc12$getMCtl(idx);
#line 82

#line 82
  return result;
#line 82
}
#line 82
# 117 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline void HplAdc12P$HplAdc12$enableConversion(void )
#line 117
{
  HplAdc12P$ADC12CTL0 |= 0x0002;
}

# 133 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/HplAdc12.nc"
inline static void Msp430Adc12ImplP$HplAdc12$enableConversion(void ){
#line 133
  HplAdc12P$HplAdc12$enableConversion();
#line 133
}
#line 133
# 631 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP$MultiChannel$default$dataReady(uint8_t id, uint16_t *buffer, uint16_t numSamples)
#line 631
{
}

# 105 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc"
inline static void Msp430Adc12ImplP$MultiChannel$dataReady(uint8_t arg_0x40858108, uint16_t *buffer, uint16_t numSamples){
#line 105
    Msp430Adc12ImplP$MultiChannel$default$dataReady(arg_0x40858108, buffer, numSamples);
#line 105
}
#line 105
# 620 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline error_t Msp430Adc12ImplP$SingleChannel$default$singleDataReady(uint8_t id, uint16_t data)
{
  return FAIL;
}

# 206 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static error_t Msp430Adc12ImplP$SingleChannel$singleDataReady(uint8_t arg_0x408593f0, uint16_t data){
#line 206
  unsigned char result;
#line 206

#line 206
  switch (arg_0x408593f0) {
#line 206
    case /*AntiTheftAppC.ParLight.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$0$ID:
#line 206
      result = AdcP$SingleChannel$singleDataReady(/*AntiTheftAppC.ParLight.AdcReadClientC*/AdcReadClientC$0$CLIENT, data);
#line 206
      break;
#line 206
    case /*AntiTheftAppC.ParLight.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC$1$ID:
#line 206
      result = AdcStreamP$SingleChannel$singleDataReady(/*AntiTheftAppC.ParLight.AdcReadStreamClientC*/AdcReadStreamClientC$0$RSCLIENT, data);
#line 206
      break;
#line 206
    default:
#line 206
      result = Msp430Adc12ImplP$SingleChannel$default$singleDataReady(arg_0x408593f0, data);
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
# 634 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP$Overflow$default$conversionTimeOverflow(uint8_t id)
#line 634
{
}

# 54 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
inline static void Msp430Adc12ImplP$Overflow$conversionTimeOverflow(uint8_t arg_0x408589f8){
#line 54
    Msp430Adc12ImplP$Overflow$default$conversionTimeOverflow(arg_0x408589f8);
#line 54
}
#line 54
# 633 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP$Overflow$default$memOverflow(uint8_t id)
#line 633
{
}

# 49 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
inline static void Msp430Adc12ImplP$Overflow$memOverflow(uint8_t arg_0x408589f8){
#line 49
    Msp430Adc12ImplP$Overflow$default$memOverflow(arg_0x408589f8);
#line 49
}
#line 49
# 524 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP$HplAdc12$conversionDone(uint16_t iv)
{
  bool overflow = FALSE;
  uint16_t *resultBuffer;

  if (iv <= 4) {
      if (iv == 2) {
        Msp430Adc12ImplP$Overflow$memOverflow(Msp430Adc12ImplP$clientID);
        }
      else {
#line 533
        Msp430Adc12ImplP$Overflow$conversionTimeOverflow(Msp430Adc12ImplP$clientID);
        }
      if (! Msp430Adc12ImplP$HplAdc12$getCtl0().msc) {
        overflow = TRUE;
        }
    }
#line 538
  switch (Msp430Adc12ImplP$state & Msp430Adc12ImplP$CONVERSION_MODE_MASK) 
    {
      case Msp430Adc12ImplP$SINGLE_DATA: 
        Msp430Adc12ImplP$stopConversion();
      Msp430Adc12ImplP$SingleChannel$singleDataReady(Msp430Adc12ImplP$clientID, Msp430Adc12ImplP$HplAdc12$getMem(0));
      break;
      case Msp430Adc12ImplP$SINGLE_DATA_REPEAT: 
        {
          error_t repeatContinue;

#line 547
          repeatContinue = Msp430Adc12ImplP$SingleChannel$singleDataReady(Msp430Adc12ImplP$clientID, 
          Msp430Adc12ImplP$HplAdc12$getMem(0));
          if (repeatContinue != SUCCESS) {
            Msp430Adc12ImplP$stopConversion();
            }
#line 551
          break;
        }

      case Msp430Adc12ImplP$MULTI_CHANNEL: 
        {
          uint16_t i = 0;
#line 556
          uint16_t k;

#line 557
          resultBuffer = Msp430Adc12ImplP$resultBufferStart;
          do {
              * resultBuffer++ = Msp430Adc12ImplP$HplAdc12$getMem(i);
            }
          while (
#line 560
          ++i < Msp430Adc12ImplP$numChannels);
          Msp430Adc12ImplP$resultBufferIndex += Msp430Adc12ImplP$numChannels;
          if (overflow || Msp430Adc12ImplP$resultBufferLength == Msp430Adc12ImplP$resultBufferIndex) {
              Msp430Adc12ImplP$stopConversion();
              resultBuffer -= Msp430Adc12ImplP$resultBufferIndex;
              k = Msp430Adc12ImplP$resultBufferIndex - Msp430Adc12ImplP$numChannels;
              Msp430Adc12ImplP$resultBufferIndex = 0;
              Msp430Adc12ImplP$MultiChannel$dataReady(Msp430Adc12ImplP$clientID, resultBuffer, 
              overflow ? k : Msp430Adc12ImplP$resultBufferLength);
            }
          else {
#line 569
            Msp430Adc12ImplP$HplAdc12$enableConversion();
            }
        }
#line 571
      break;
      case Msp430Adc12ImplP$MULTIPLE_DATA: 
        {
          uint16_t i = 0;
#line 574
          uint16_t length;
#line 574
          uint16_t k;

#line 575
          resultBuffer = Msp430Adc12ImplP$resultBufferStart + Msp430Adc12ImplP$resultBufferIndex;
          if (Msp430Adc12ImplP$resultBufferLength - Msp430Adc12ImplP$resultBufferIndex > 16) {
            length = 16;
            }
          else {
#line 579
            length = Msp430Adc12ImplP$resultBufferLength - Msp430Adc12ImplP$resultBufferIndex;
            }
#line 580
          do {
              * resultBuffer++ = Msp430Adc12ImplP$HplAdc12$getMem(i);
            }
          while (
#line 582
          ++i < length);
          Msp430Adc12ImplP$resultBufferIndex += length;
          if (overflow || Msp430Adc12ImplP$resultBufferLength == Msp430Adc12ImplP$resultBufferIndex) {
              Msp430Adc12ImplP$stopConversion();
              resultBuffer -= Msp430Adc12ImplP$resultBufferIndex;
              k = Msp430Adc12ImplP$resultBufferIndex - length;
              Msp430Adc12ImplP$resultBufferIndex = 0;
              Msp430Adc12ImplP$SingleChannel$multipleDataReady(Msp430Adc12ImplP$clientID, resultBuffer, 
              overflow ? k : Msp430Adc12ImplP$resultBufferLength);
            }
          else {
#line 591
            if (Msp430Adc12ImplP$resultBufferLength - Msp430Adc12ImplP$resultBufferIndex > 15) {
              return;
              }
            else 
#line 593
              {

                adc12memctl_t memctl = Msp430Adc12ImplP$HplAdc12$getMCtl(0);

#line 596
                memctl.eos = 1;
                Msp430Adc12ImplP$HplAdc12$setMCtl(Msp430Adc12ImplP$resultBufferLength - Msp430Adc12ImplP$resultBufferIndex, memctl);
              }
            }
        }
#line 600
      break;
      case Msp430Adc12ImplP$MULTIPLE_DATA_REPEAT: 
        {
          uint8_t i = 0;

#line 604
          resultBuffer = Msp430Adc12ImplP$resultBufferStart;
          do {
              * resultBuffer++ = Msp430Adc12ImplP$HplAdc12$getMem(i);
            }
          while (
#line 607
          ++i < Msp430Adc12ImplP$resultBufferLength);

          Msp430Adc12ImplP$resultBufferStart = Msp430Adc12ImplP$SingleChannel$multipleDataReady(Msp430Adc12ImplP$clientID, 
          resultBuffer - Msp430Adc12ImplP$resultBufferLength, 
          overflow ? 0 : Msp430Adc12ImplP$resultBufferLength);
          if (!Msp430Adc12ImplP$resultBufferStart) {
            Msp430Adc12ImplP$stopConversion();
            }
#line 614
          break;
        }
    }
}

# 274 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline void Msp430RefVoltGeneratorP$HplAdc12$conversionDone(uint16_t iv)
#line 274
{
}

# 112 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/HplAdc12.nc"
inline static void HplAdc12P$HplAdc12$conversionDone(uint16_t iv){
#line 112
  Msp430RefVoltGeneratorP$HplAdc12$conversionDone(iv);
#line 112
  Msp430Adc12ImplP$HplAdc12$conversionDone(iv);
#line 112
}
#line 112
# 56 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )55U &= ~(0x01 << 0);
}

# 85 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP$Port60$selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP$40$IO$selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )55U &= ~(0x01 << 1);
}

# 85 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP$Port61$selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP$41$IO$selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )55U &= ~(0x01 << 2);
}

# 85 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP$Port62$selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP$42$IO$selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )55U &= ~(0x01 << 3);
}

# 85 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP$Port63$selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP$43$IO$selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )55U &= ~(0x01 << 4);
}

# 85 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP$Port64$selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP$44$IO$selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )55U &= ~(0x01 << 5);
}

# 85 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP$Port65$selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP$45$IO$selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )55U &= ~(0x01 << 6);
}

# 85 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP$Port66$selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP$46$IO$selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )55U &= ~(0x01 << 7);
}

# 85 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP$Port67$selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP$47$IO$selectIOFunc();
#line 85
}
#line 85
# 98 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline void HplAdc12P$HplAdc12$resetIFGs(void )
#line 98
{
  HplAdc12P$ADC12IV = 0;
  HplAdc12P$ADC12IFG = 0;
}

# 106 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/HplAdc12.nc"
inline static void Msp430Adc12ImplP$HplAdc12$resetIFGs(void ){
#line 106
  HplAdc12P$HplAdc12$resetIFGs();
#line 106
}
#line 106
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
inline static error_t AdcStreamP$readStreamFail$postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(AdcStreamP$readStreamFail);
#line 56

#line 56
  return result;
#line 56
}
#line 56
inline static error_t AdcStreamP$bufferDone$postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(AdcStreamP$bufferDone);
#line 56

#line 56
  return result;
#line 56
}
#line 56
inline static error_t AdcP$readDone$postTask(void ){
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
# 178 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcP.nc"
static inline void AdcP$ReadNow$default$readDone(uint8_t client, error_t result, uint16_t val)
#line 178
{
}

# 66 "/opt/tinyos-2.1.0/tos/interfaces/ReadNow.nc"
inline static void AdcP$ReadNow$readDone(uint8_t arg_0x40818d70, error_t result, AdcP$ReadNow$val_t val){
#line 66
    AdcP$ReadNow$default$readDone(arg_0x40818d70, result, val);
#line 66
}
#line 66
# 226 "/opt/tinyos-2.1.0/tos/chips/msp430/msp430hardware.h"
  __nesc_atomic_t __nesc_atomic_start(void )
{
  __nesc_atomic_t result = (({
#line 228
    uint16_t __x;

#line 228
     __asm volatile ("mov	r2, %0" : "=r"((uint16_t )__x));__x;
  }
  )
#line 228
   & 0x0008) != 0;

#line 229
  __nesc_disable_interrupt();
   __asm volatile ("" :  :  : "memory");
  return result;
}

  void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)
{
   __asm volatile ("" :  :  : "memory");
  if (reenable_interrupts) {
    __nesc_enable_interrupt();
    }
}

# 11 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(12)))  void sig_TIMERA0_VECTOR(void )
#line 11
{
#line 11
  Msp430TimerCommonP$VectorTimerA0$fired();
}

# 169 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Event$fired(void )
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
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Event$fired(void )
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
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Event$fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$captured(/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$fired();
    }
}

# 12 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(10)))  void sig_TIMERA1_VECTOR(void )
#line 12
{
#line 12
  Msp430TimerCommonP$VectorTimerA1$fired();
}

#line 13
__attribute((wakeup)) __attribute((interrupt(26)))  void sig_TIMERB0_VECTOR(void )
#line 13
{
#line 13
  Msp430TimerCommonP$VectorTimerB0$fired();
}

# 135 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerP.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$default$fired(uint8_t n)
{
}

# 28 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$fired(uint8_t arg_0x40615428){
#line 28
  switch (arg_0x40615428) {
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
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$default$fired(arg_0x40615428);
#line 28
      break;
#line 28
    }
#line 28
}
#line 28
# 159 "/opt/tinyos-2.1.0/tos/system/SchedulerBasicP.nc"
static error_t SchedulerBasicP$TaskBasic$postTask(uint8_t id)
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

# 96 "/opt/tinyos-2.1.0/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$set_alarm(void )
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

# 69 "/opt/tinyos-2.1.0/tos/lib/timer/TransformCounterC.nc"
static /*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$get(void )
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

# 51 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$get(void )
{




  if (1) {
      /* atomic removed: atomic calls only */
#line 58
      {
        uint16_t t0;
        uint16_t t1 = * (volatile uint16_t * )400U;

#line 61
        do {
#line 61
            t0 = t1;
#line 61
            t1 = * (volatile uint16_t * )400U;
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
      return * (volatile uint16_t * )400U;
    }
}

# 378 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static error_t Msp430Adc12ImplP$SingleChannel$getData(uint8_t id)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 380
    {
      if (Msp430Adc12ImplP$ADCArbiterInfo$userId() == id) {
          if (Msp430Adc12ImplP$state & Msp430Adc12ImplP$MULTIPLE_DATA_REPEAT && !Msp430Adc12ImplP$resultBufferStart) 
            {
              unsigned char __nesc_temp = 
#line 383
              EINVAL;

              {
#line 383
                __nesc_atomic_end(__nesc_atomic); 
#line 383
                return __nesc_temp;
              }
            }
#line 384
          if (Msp430Adc12ImplP$state & Msp430Adc12ImplP$ADC_BUSY) 
            {
              unsigned char __nesc_temp = 
#line 385
              EBUSY;

              {
#line 385
                __nesc_atomic_end(__nesc_atomic); 
#line 385
                return __nesc_temp;
              }
            }
#line 386
          Msp430Adc12ImplP$state |= Msp430Adc12ImplP$ADC_BUSY;
          Msp430Adc12ImplP$clientID = id;
          Msp430Adc12ImplP$configureAdcPin(Msp430Adc12ImplP$HplAdc12$getMCtl(0).inch);
          Msp430Adc12ImplP$HplAdc12$startConversion();
          if (Msp430Adc12ImplP$state & Msp430Adc12ImplP$USE_TIMERA) {
            Msp430Adc12ImplP$startTimerA();
            }
#line 392
          {
            unsigned char __nesc_temp = 
#line 392
            SUCCESS;

            {
#line 392
              __nesc_atomic_end(__nesc_atomic); 
#line 392
              return __nesc_temp;
            }
          }
        }
    }
#line 396
    __nesc_atomic_end(__nesc_atomic); }
#line 395
  return FAIL;
}

# 136 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
static uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$ArbiterInfo$userId(void )
#line 136
{
  /* atomic removed: atomic calls only */
#line 137
  {
    if (/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$state != /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$RES_BUSY) 
      {
        unsigned char __nesc_temp = 
#line 139
        /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$NO_RES;

#line 139
        return __nesc_temp;
      }
#line 140
    {
      unsigned char __nesc_temp = 
#line 140
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$resId;

#line 140
      return __nesc_temp;
    }
  }
}

# 83 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/HplAdc12P.nc"
static adc12memctl_t HplAdc12P$HplAdc12$getMCtl(uint8_t i)
#line 83
{
  adc12memctl_t x = { .inch = 0, .sref = 0, .eos = 0 };
  uint8_t *memCtlPtr = (uint8_t *)(char *)0x0080;

#line 86
  memCtlPtr += i;
  x = * (adc12memctl_t *)memCtlPtr;
  return x;
}

# 80 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerP.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$setMode(int mode)
{
  * (volatile uint16_t * )352U = (* (volatile uint16_t * )352U & ~(0x0020 | 0x0010)) | ((mode << 4) & (0x0020 | 0x0010));
}

# 96 "/opt/tinyos-2.1.0/tos/lib/timer/TransformAlarmC.nc"
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$set_alarm(void )
{
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$to_size_type now = /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$Counter$get();
#line 98
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$to_size_type expires;
#line 98
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$to_size_type remaining;




  expires = /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$m_t0 + /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$m_dt;


  remaining = (/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$to_size_type )(expires - now);


  if (/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$m_t0 <= now) 
    {
      if (expires >= /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 121
  if (remaining > /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$MAX_DELAY) 
    {
      /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$m_t0 = now + /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$MAX_DELAY;
      /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$m_dt = remaining - /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$MAX_DELAY;
      remaining = /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$MAX_DELAY;
    }
  else 
    {
      /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$m_t0 += /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$m_dt;
      /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$m_dt = 0;
    }
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$AlarmFrom$startAt((/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$from_size_type )now << 5, 
  (/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC$1$from_size_type )remaining << 5);
}

# 14 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(24)))  void sig_TIMERB1_VECTOR(void )
#line 14
{
#line 14
  Msp430TimerCommonP$VectorTimerB1$fired();
}

# 52 "/opt/tinyos-2.1.0/tos/system/RealMainP.nc"
  int main(void )
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

# 160 "/opt/tinyos-2.1.0/tos/chips/msp430/timer/Msp430ClockP.nc"
static void Msp430ClockP$set_dco_calib(int calib)
{
  BCSCTL1 = (BCSCTL1 & ~0x07) | ((calib >> 8) & 0x07);
  DCOCTL = calib & 0xff;
}

# 16 "/opt/tinyos-2.1.0/tos/platforms/telosb/MotePlatformC.nc"
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

# 45 "/opt/tinyos-2.1.0/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$set(void )
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t * )49U |= 0x01 << 6;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

# 123 "/opt/tinyos-2.1.0/tos/system/SchedulerBasicP.nc"
static bool SchedulerBasicP$Scheduler$runNextTask(void )
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
static void SchedulerBasicP$TaskBasic$default$runTask(uint8_t id)
{
}

# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP$TaskBasic$runTask(uint8_t arg_0x405744a0){
#line 64
  switch (arg_0x405744a0) {
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
    case AdcStreamP$readStreamDone:
#line 64
      AdcStreamP$readStreamDone$runTask();
#line 64
      break;
#line 64
    case AdcStreamP$readStreamFail:
#line 64
      AdcStreamP$readStreamFail$runTask();
#line 64
      break;
#line 64
    case AdcStreamP$bufferDone:
#line 64
      AdcStreamP$bufferDone$runTask();
#line 64
      break;
#line 64
    default:
#line 64
      SchedulerBasicP$TaskBasic$default$runTask(arg_0x405744a0);
#line 64
      break;
#line 64
    }
#line 64
}
#line 64
# 116 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static error_t Msp430RefVoltArbiterImplP$ClientResource$release(uint8_t client)
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

# 97 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$release(uint8_t id)
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

# 65 "/opt/tinyos-2.1.0/tos/system/RoundRobinResourceQueueC.nc"
static bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$RoundRobinQueue$isEnqueued(resource_client_id_t id)
#line 65
{
  return /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC$0$resQ[id / 8] & (1 << id % 8);
}

# 62 "/opt/tinyos-2.1.0/tos/lib/timer/VirtualizeTimerC.nc"
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
              break;
            }
        }
    }
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$postTask();
}

# 71 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$request(uint8_t id)
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

# 78 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static error_t Msp430RefVoltGeneratorP$switchOff(void )
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

# 148 "/opt/tinyos-2.1.0/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShot(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow(), dt, TRUE);
}

#line 133
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_timers[num];

#line 136
  timer->t0 = t0;
  timer->dt = dt;
  timer->isoneshot = isoneshot;
  timer->isrunning = TRUE;
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$postTask();
}

# 212 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcStreamP.nc"
static error_t AdcStreamP$ReadStream$read(uint8_t c, uint32_t usPeriod)
{
  if (usPeriod & 0xFFFF0000) {

      AdcStreamP$period = usPeriod / 1000;
      AdcStreamP$periodModified = TRUE;
      AdcStreamP$client = c;
      AdcStreamP$now = AdcStreamP$Alarm$getNow();
      AdcStreamP$SingleChannel$configureSingle(c, AdcStreamP$AdcConfigure$getConfiguration(c));
      if (AdcStreamP$nextBuffer(FALSE) == SUCCESS) {
        AdcStreamP$sampleSingle();
        }
    }
  else 
#line 223
    {
      AdcStreamP$period = usPeriod;
      AdcStreamP$periodModified = FALSE;
      AdcStreamP$client = c;
      AdcStreamP$nextMultiple(c);
    }
  return SUCCESS;
}

# 172 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static error_t Msp430Adc12ImplP$SingleChannel$configureSingle(uint8_t id, 
const msp430adc12_channel_config_t *config)
{
  error_t result = ERESERVE;

  if (!config) {
    return EINVAL;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 180
    {
      if (Msp430Adc12ImplP$state & Msp430Adc12ImplP$ADC_BUSY) 
        {
          unsigned char __nesc_temp = 
#line 182
          EBUSY;

          {
#line 182
            __nesc_atomic_end(__nesc_atomic); 
#line 182
            return __nesc_temp;
          }
        }
#line 183
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

#line 200
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
#line 211
    __nesc_atomic_end(__nesc_atomic); }
  return result;
}

# 172 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcStreamP.nc"
static error_t AdcStreamP$nextBuffer(bool startNextAlarm)
#line 172
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      struct AdcStreamP$list_entry_t *entry = AdcStreamP$bufferQueue[AdcStreamP$client];

      if (!entry) 
        {

          AdcStreamP$bufferQueueEnd[AdcStreamP$client] = (void *)0;
          AdcStreamP$readStreamDone$postTask();
          {
            unsigned char __nesc_temp = 
#line 182
            FAIL;

            {
#line 182
              __nesc_atomic_end(__nesc_atomic); 
#line 182
              return __nesc_temp;
            }
          }
        }
      else 
#line 185
        {
          uint16_t tmp_count;

#line 187
          AdcStreamP$bufferQueue[AdcStreamP$client] = entry->next;
          if (!AdcStreamP$bufferQueue[AdcStreamP$client]) {
            AdcStreamP$bufferQueueEnd[AdcStreamP$client] = &AdcStreamP$bufferQueue[AdcStreamP$client];
            }
#line 190
          AdcStreamP$pos = AdcStreamP$buffer = (void *)0;
          AdcStreamP$count = entry->count;
          tmp_count = AdcStreamP$count;
          AdcStreamP$pos = AdcStreamP$buffer = (uint16_t * )entry;
          if (startNextAlarm) {
            AdcStreamP$nextAlarm();
            }
#line 196
          {
            unsigned char __nesc_temp = 
#line 196
            SUCCESS;

            {
#line 196
              __nesc_atomic_end(__nesc_atomic); 
#line 196
              return __nesc_temp;
            }
          }
        }
    }
#line 200
    __nesc_atomic_end(__nesc_atomic); }
}

#line 201
static void AdcStreamP$nextMultiple(uint8_t c)
{
  if (AdcStreamP$nextBuffer(FALSE) == SUCCESS) {
      msp430adc12_channel_config_t config = *AdcStreamP$AdcConfigure$getConfiguration(c);

#line 205
      config.sampcon_ssel = SAMPCON_SOURCE_SMCLK;
      config.sampcon_id = SAMPCON_CLOCK_DIV_1;
      AdcStreamP$SingleChannel$configureMultiple(c, &config, AdcStreamP$pos, AdcStreamP$count, AdcStreamP$period);
      AdcStreamP$SingleChannel$getData(c);
    }
}

# 80 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcP.nc"
static void AdcP$ResourceRead$granted(uint8_t client)
{

  error_t result = AdcP$configure(client);

#line 84
  if (result == SUCCESS) {
      AdcP$state = AdcP$STATE_READ;
      result = AdcP$SingleChannel$getData(client);
    }
  else 
#line 87
    {
      AdcP$ResourceRead$release(client);
      AdcP$Read$readDone(client, result, 0);
    }
}

# 33 "DarkC.nc"
static void DarkC$Light$readDone(error_t ok, uint16_t val)
{
  if (ok == SUCCESS && val < DarkC$DARK_THRESHOLD) {
    DarkC$Leds$led2On();
    }
  else {
#line 38
    DarkC$Leds$led2Off();
    }
}

# 107 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static void Msp430RefVoltArbiterImplP$RefVolt_2_5V$startDone(error_t error)
{
  if (Msp430RefVoltArbiterImplP$syncOwner != Msp430RefVoltArbiterImplP$NO_OWNER) {


      Msp430RefVoltArbiterImplP$ClientResource$granted(Msp430RefVoltArbiterImplP$syncOwner);
    }
}

# 136 "/opt/tinyos-2.1.0/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type dt)
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

# 70 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static void Msp430RefVoltArbiterImplP$AdcResource$granted(uint8_t client)
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

# 58 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
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

# 108 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/HplAdc12P.nc"
static void HplAdc12P$HplAdc12$stopConversion(void )
#line 108
{

  uint16_t ctl1 = HplAdc12P$ADC12CTL1;

#line 111
  HplAdc12P$ADC12CTL1 &= ~(0x0002 | 0x0004);
  HplAdc12P$ADC12CTL0 &= ~(0x0001 + 0x0002);
  HplAdc12P$ADC12CTL0 &= ~0x0010;
  HplAdc12P$ADC12CTL1 |= ctl1 & (0x0002 | 0x0004);
}







__attribute((wakeup)) __attribute((interrupt(14)))  void sig_ADC_VECTOR(void )
#line 123
{
  HplAdc12P$HplAdc12$conversionDone(HplAdc12P$ADC12IV);
}

# 483 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static void Msp430Adc12ImplP$stopConversion(void )
{
  uint8_t i;

  if (Msp430Adc12ImplP$state & Msp430Adc12ImplP$USE_TIMERA) {
    Msp430Adc12ImplP$TimerA$setMode(MSP430TIMER_STOP_MODE);
    }
  Msp430Adc12ImplP$resetAdcPin(Msp430Adc12ImplP$HplAdc12$getMCtl(0).inch);
  if (Msp430Adc12ImplP$state & Msp430Adc12ImplP$MULTI_CHANNEL) {
      for (i = 1; i < Msp430Adc12ImplP$numChannels; i++) 
        Msp430Adc12ImplP$resetAdcPin(Msp430Adc12ImplP$HplAdc12$getMCtl(i).inch);
    }
  /* atomic removed: atomic calls only */
#line 495
  {
    Msp430Adc12ImplP$HplAdc12$stopConversion();
    Msp430Adc12ImplP$HplAdc12$resetIFGs();
    Msp430Adc12ImplP$state &= ~Msp430Adc12ImplP$ADC_BUSY;
  }
}

#line 155
static void Msp430Adc12ImplP$resetAdcPin(uint8_t inch)
{

  switch (inch) 
    {
      case 0: Msp430Adc12ImplP$Port60$selectIOFunc();
#line 160
      break;
      case 1: Msp430Adc12ImplP$Port61$selectIOFunc();
#line 161
      break;
      case 2: Msp430Adc12ImplP$Port62$selectIOFunc();
#line 162
      break;
      case 3: Msp430Adc12ImplP$Port63$selectIOFunc();
#line 163
      break;
      case 4: Msp430Adc12ImplP$Port64$selectIOFunc();
#line 164
      break;
      case 5: Msp430Adc12ImplP$Port65$selectIOFunc();
#line 165
      break;
      case 6: Msp430Adc12ImplP$Port66$selectIOFunc();
#line 166
      break;
      case 7: Msp430Adc12ImplP$Port67$selectIOFunc();
#line 167
      break;
    }
}

# 233 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcStreamP.nc"
static error_t AdcStreamP$SingleChannel$singleDataReady(uint8_t streamClient, uint16_t data)
{
  if (AdcStreamP$client == AdcStreamP$NSTREAM) {
    return FAIL;
    }
  if (AdcStreamP$count == 0) 
    {
      AdcStreamP$now = AdcStreamP$Alarm$getNow();
      AdcStreamP$nextBuffer(TRUE);
    }
  else 
    {
      * AdcStreamP$pos++ = data;
      if (AdcStreamP$pos == AdcStreamP$buffer + AdcStreamP$count) 
        {
          /* atomic removed: atomic calls only */
          {
            if (AdcStreamP$lastBuffer) 
              {

                AdcStreamP$bufferQueueEnd[AdcStreamP$client] = (void *)0;
                AdcStreamP$readStreamFail$postTask();
                {
                  unsigned char __nesc_temp = 
#line 255
                  FAIL;

#line 255
                  return __nesc_temp;
                }
              }
            else {
                AdcStreamP$lastCount = AdcStreamP$count;
                AdcStreamP$lastBuffer = AdcStreamP$buffer;
              }
          }
          AdcStreamP$bufferDone$postTask();
          AdcStreamP$nextBuffer(TRUE);
        }
      else {
        AdcStreamP$nextAlarm();
        }
    }
#line 269
  return FAIL;
}

# 142 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcP.nc"
static error_t AdcP$SingleChannel$singleDataReady(uint8_t client, uint16_t data)
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

# 272 "/opt/tinyos-2.1.0/tos/chips/msp430/adc12/AdcStreamP.nc"
static uint16_t *AdcStreamP$SingleChannel$multipleDataReady(uint8_t streamClient, 
uint16_t *buf, uint16_t length)
{
  /* atomic removed: atomic calls only */
  {
    if (AdcStreamP$lastBuffer) 
      {

        AdcStreamP$bufferQueueEnd[AdcStreamP$client] = (void *)0;
        AdcStreamP$readStreamFail$postTask();
        {
          unsigned int *__nesc_temp = 
#line 282
          0;

#line 282
          return __nesc_temp;
        }
      }
    else {
        AdcStreamP$lastBuffer = AdcStreamP$buffer;
        AdcStreamP$lastCount = AdcStreamP$pos - AdcStreamP$buffer;
      }
  }
  AdcStreamP$bufferDone$postTask();
  AdcStreamP$nextMultiple(streamClient);
  return 0;
}

