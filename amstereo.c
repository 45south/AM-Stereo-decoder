/*
 * amstereo.c - AM Stereo Decoder (C-QUAM / Kahn-Hazeltine ISB)
 * Win32 GUI - single exe
 *
 * Build: make
 * SDRuno: VRX mode = IQ OUT, output = VAC, tuned to AM station carrier
 */

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <mmsystem.h>
#include <commctrl.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#ifndef WAVE_FORMAT_IEEE_FLOAT
#define WAVE_FORMAT_IEEE_FLOAT 0x0003
#endif

/* ── Control IDs ─────────────────────────────────────────────────────────── */
#define IDC_DEV_IN       101
#define IDC_DEV_OUT      102
#define IDC_MODE_CQUAM   103
#define IDC_MODE_KAHN    104
#define IDC_BW_DX        105
#define IDC_BW_NARROW    106
#define IDC_BW_NORMAL    107
#define IDC_BW_WIDE      108
#define IDC_BW_CUSTOM    109
#define IDC_BW_HZ        110
#define IDC_GAIN_SLIDER  111
#define IDC_GAIN_LABEL   112
#define IDC_BTN_START    113
#define IDC_BTN_STOP     114
#define IDC_METER_L      115
#define IDC_METER_R      116
#define IDC_STEREO_LED   117
#define IDC_TIMER        118
#define IDC_BTN_MONO     119
#define IDC_BTN_ONTOP    120
#define IDC_NOTCH_OFF    121
#define IDC_NOTCH_9K     122
#define IDC_NOTCH_10K    123
#define IDC_VNOTCH_CHK   124
#define IDC_VNOTCH_SLD   125
#define IDC_VNOTCH_LABEL 126
#define IDC_BTN_ANTIFADE 127

/* ── Decoder constants ───────────────────────────────────────────────────── */
#define CHANNELS_IQ      2
#define BLOCK_FRAMES     4096
#define NUM_BUFFERS      4
#define AUDIO_RATE       48000
#define AUDIO_BITS       16
#define AUDIO_CHANNELS   2
#define IQ_LPF_TAPS      255
#define PLL_ALPHA        0.1
#define PLL_BETA         0.001
#define AGC_ATTACK       0.01
#define AGC_DECAY        0.0001
#define AGC_TARGET       4000.0
#define AGC_MAX_GAIN     500.0
#define AGC_MIN_GAIN     0.001
#define CLIP_KNEE        16000.0
#define LOCK_THRESHOLD   0.15
#define LOCK_AVERAGE     0.99995
#define LOCK_MIN_ENV     500.0
#define DC_R             0.9995
#define BLOCK_BYTES_F32  (BLOCK_FRAMES * CHANNELS_IQ * 4)
#define BLOCK_BYTES_I16  (BLOCK_FRAMES * CHANNELS_IQ * 2)
#define BLOCK_BYTES_MAX  BLOCK_BYTES_F32
#define OUT_SAMPLES      ((BLOCK_FRAMES + 16) * AUDIO_CHANNELS)
#define GAIN_SLIDER_MIN  1
#define GAIN_SLIDER_MAX  100
#define GAIN_MIN         0.1
#define GAIN_MAX         10.0

typedef enum { FMT_INT16, FMT_FLOAT32 } InputFmt;
typedef enum { MODE_CQUAM, MODE_KAHN  } DecodeMode;

/* ── Shared state ────────────────────────────────────────────────────────── */
typedef struct {
    CRITICAL_SECTION cs;
    volatile double     bw_hz;
    volatile double     gain;
    volatile DecodeMode mode;
    volatile int        force_mono;
    volatile int        antifade;    /* 1 = Kahn anti-fade blend active */
    volatile double     notch_hz;
    volatile double     vnotch_hz;
    volatile int        cmd_stop;
    volatile double     level_l;
    volatile double     level_r;
    volatile int        stereo_lock;
    volatile int        running;
} SharedState;

static SharedState g_state;

/* ── Decoder globals ─────────────────────────────────────────────────────── */
static HWAVEIN   g_hWaveIn  = NULL;
static HWAVEOUT  g_hWaveOut = NULL;
static WAVEHDR   g_in_hdr[NUM_BUFFERS];
static WAVEHDR   g_out_hdr[NUM_BUFFERS];
static uint8_t   g_in_buf[NUM_BUFFERS][BLOCK_BYTES_MAX];
static int16_t   g_out_buf[NUM_BUFFERS][OUT_SAMPLES];
static HANDLE    g_hEventIn;
static HANDLE    g_hEventOut;
static DWORD     g_sample_rate = 0;
static InputFmt  g_fmt         = FMT_FLOAT32;

static double g_lpf[IQ_LPF_TAPS];
static double g_zi[IQ_LPF_TAPS];
static double g_zq[IQ_LPF_TAPS];
static double g_zl[IQ_LPF_TAPS];
static double g_zr[IQ_LPF_TAPS];
static double g_zlpr[IQ_LPF_TAPS];
static double g_zlmr[IQ_LPF_TAPS];

static double g_dc_x1=0,g_dc_y1=0;
static double g_pll_phase=0,g_pll_freq=0;
static double g_agc_gain=1.0;
static double g_lock_level=0;
static int    g_out_idx=0;
static double g_antifade_blend=1.0;
static double g_af_dc_x1=0, g_af_dc_y1=0;

/* Click suppression - mute on sudden AGC jumps */
static double g_last_agc_gain=1.0;
static int    g_mute_samples=0;
static double g_pll_err_avg=1.0; /* smoothed PLL error for lock detection */
#define MUTE_THRESHOLD  3.0    /* AGC gain ratio that triggers mute */
#define MUTE_HOLD       9600   /* ~100ms at 96kHz base mute */
#define MUTE_PLL_EXT    48000  /* up to 500ms extra while PLL settling */ /* separate DC blocker for antifade */ /* 1=full stereo, 0=mono */

/* ── Notch ───────────────────────────────────────────────────────────────── */
typedef struct { double x1,x2,y1,y2; } NS;
static NS g_nl1={0},g_nr1={0},g_nl2={0},g_nr2={0};
static NS g_vl1={0},g_vr1={0},g_vl2={0},g_vr2={0};
static double g_nb0=1,g_nb1=0,g_nb2=1,g_na1=0,g_na2=0,g_nf=0;
static double g_vb0=1,g_vb1=0,g_vb2=1,g_va1=0,g_va2=0,g_vf=0;

/* ── DSP ─────────────────────────────────────────────────────────────────── */
static double dc_block(double x)
{
    double y=x-g_dc_x1+DC_R*g_dc_y1;
    g_dc_x1=x; g_dc_y1=y; return y;
}
static double soft_clip(double x)
{
    double ax=fabs(x);
    if(ax<=CLIP_KNEE) return x;
    double o=ax-CLIP_KNEE,r=32767.0-CLIP_KNEE;
    return (x>0)?(CLIP_KNEE+r*tanh(o/r)):-(CLIP_KNEE+r*tanh(o/r));
}
static void build_lpf(double *c,int taps,double cut,double fs)
{
    int M=taps-1; double fc=cut/fs,sum=0;
    for(int n=0;n<=M;n++){
        double h=(n==M/2)?2*fc:sin(2*M_PI*fc*(n-M/2.0))/(M_PI*(n-M/2.0));
        double w=0.5*(1-cos(2*M_PI*n/M));
        c[n]=h*w; sum+=c[n];
    }
    for(int n=0;n<taps;n++) c[n]/=sum;
}
static double fir(double *c,int taps,double *z,double x)
{
    memmove(&z[1],&z[0],(taps-1)*sizeof(double));
    z[0]=x; double y=0;
    for(int k=0;k<taps;k++) y+=c[k]*z[k];
    return y;
}
static void mk_notch(double f,double fs,double Q,
    double *b0,double *b1,double *b2,double *a1,double *a2)
{
    double w=2*M_PI*f/fs,al=sin(w)/(2*Q),a0=1+al;
    *b0=1/a0; *b1=-2*cos(w)/a0; *b2=1/a0;
    *a1=-2*cos(w)/a0; *a2=(1-al)/a0;
}
static void set_notch(double f,double fs)
{
    if(f<=0){g_nf=0;return;}
    mk_notch(f,fs,200,&g_nb0,&g_nb1,&g_nb2,&g_na1,&g_na2);
    g_nf=f;
    memset(&g_nl1,0,sizeof(NS));memset(&g_nr1,0,sizeof(NS));
    memset(&g_nl2,0,sizeof(NS));memset(&g_nr2,0,sizeof(NS));
}
static void set_vnotch(double f,double fs)
{
    if(f<=0){g_vf=0;return;}
    mk_notch(f,fs,30,&g_vb0,&g_vb1,&g_vb2,&g_va1,&g_va2);
    g_vf=f;
    memset(&g_vl1,0,sizeof(NS));memset(&g_vr1,0,sizeof(NS));
    memset(&g_vl2,0,sizeof(NS));memset(&g_vr2,0,sizeof(NS));
}
static double ntick(NS *s,double b0,double b1,double b2,double a1,double a2,double x)
{
    double y=b0*x+b1*s->x1+b2*s->x2-a1*s->y1-a2*s->y2;
    s->x2=s->x1;s->x1=x;s->y2=s->y1;s->y1=y;return y;
}

/* ── Buffer helpers ──────────────────────────────────────────────────────── */
static void requeue_in(int i)
{
    g_in_hdr[i].dwBufferLength=(g_fmt==FMT_FLOAT32)?BLOCK_BYTES_F32:BLOCK_BYTES_I16;
    g_in_hdr[i].dwFlags=0;
    waveInPrepareHeader(g_hWaveIn,&g_in_hdr[i],sizeof(WAVEHDR));
    waveInAddBuffer(g_hWaveIn,&g_in_hdr[i],sizeof(WAVEHDR));
}
static void submit_out(int i,DWORD s)
{
    g_out_hdr[i].dwBufferLength=s*(AUDIO_BITS/8);
    g_out_hdr[i].dwFlags=0;
    waveOutPrepareHeader(g_hWaveOut,&g_out_hdr[i],sizeof(WAVEHDR));
    waveOutWrite(g_hWaveOut,&g_out_hdr[i],sizeof(WAVEHDR));
}

/* ── Process one IQ block ────────────────────────────────────────────────── */
static void process_block(const uint8_t *raw,DWORD frames,
                          double gain,DecodeMode mode,int fmono,int antifade)
{
    double ratio=(double)g_sample_rate/(double)AUDIO_RATE;
    double pacc=0; DWORD oc=0;
    double pl=0,pr=0;

    while(g_out_hdr[g_out_idx].dwFlags&WHDR_INQUEUE)
        WaitForSingleObject(g_hEventOut,10);
    if(g_out_hdr[g_out_idx].dwFlags&WHDR_PREPARED)
        waveOutUnprepareHeader(g_hWaveOut,&g_out_hdr[g_out_idx],sizeof(WAVEHDR));
    int16_t *out=g_out_buf[g_out_idx];

    for(DWORD f=0;f<frames;f++){
        double ir,qr;
        if(g_fmt==FMT_FLOAT32){
            float fi,fq;
            memcpy(&fi,raw+f*8,4);memcpy(&fq,raw+f*8+4,4);
            ir=fi*32768.0;qr=fq*32768.0;
        }else{
            const int16_t *s=(const int16_t*)raw;
            ir=s[f*2];qr=s[f*2+1];
        }

        /* IQ LPF */
        double i_f=fir(g_lpf,IQ_LPF_TAPS,g_zi,ir);
        double q_f=fir(g_lpf,IQ_LPF_TAPS,g_zq,qr);
        double env=sqrt(i_f*i_f+q_f*q_f);

        /* AGC */
        if(env*g_agc_gain>AGC_TARGET) g_agc_gain*=(1.0-AGC_ATTACK);
        else                           g_agc_gain*=(1.0+AGC_DECAY);
        if(g_agc_gain>AGC_MAX_GAIN) g_agc_gain=AGC_MAX_GAIN;
        if(g_agc_gain<AGC_MIN_GAIN) g_agc_gain=AGC_MIN_GAIN;

        /* Click suppression: mute on sudden AGC jump (retune) */
        double agc_ratio = g_agc_gain / (g_last_agc_gain + 1e-10);
        if(agc_ratio > MUTE_THRESHOLD || agc_ratio < 1.0/MUTE_THRESHOLD) {
            g_mute_samples = MUTE_HOLD;
            /* Reset filter states to avoid click propagating */
            memset(g_zi,  0, IQ_LPF_TAPS*sizeof(double));
            memset(g_zq,  0, IQ_LPF_TAPS*sizeof(double));
            memset(g_zl,  0, IQ_LPF_TAPS*sizeof(double));
            memset(g_zr,  0, IQ_LPF_TAPS*sizeof(double));
            memset(g_zlpr,0, IQ_LPF_TAPS*sizeof(double));
            memset(g_zlmr,0, IQ_LPF_TAPS*sizeof(double));
            g_dc_x1=g_dc_y1=0;
            g_af_dc_x1=g_af_dc_y1=0;
            g_pll_phase=g_pll_freq=0;
        }
        g_last_agc_gain = g_agc_gain;
        if(g_mute_samples > 0) g_mute_samples--;

        double ia=i_f*g_agc_gain, qa=q_f*g_agc_gain, ea=env*g_agc_gain;
        double left=0,right=0;

        if(mode==MODE_CQUAM){
            double lpr=dc_block(ea);
            double cp=cos(g_pll_phase),sp=sin(g_pll_phase);
            double ir2= ia*cp+qa*sp;
            double qr2=-ia*sp+qa*cp;
            double err=0;
            if(fabs(ir2)>1.0) err=qr2/ir2;
            if(err> 1.0) err= 1.0;
            if(err<-1.0) err=-1.0;
            g_pll_freq +=PLL_BETA*err;
            g_pll_phase+=g_pll_freq+PLL_ALPHA*err;
            while(g_pll_phase> M_PI) g_pll_phase-=2*M_PI;
            while(g_pll_phase<-M_PI) g_pll_phase+=2*M_PI;
            /* Track PLL error - extend mute while not locked */
            g_pll_err_avg = 0.999*g_pll_err_avg + 0.001*fabs(err);
            if(g_mute_samples>0 && g_pll_err_avg>0.01 && g_mute_samples<MUTE_PLL_EXT)
                g_mute_samples++; /* keep muted while PLL still settling */
            double lmr=(ea>64.0)?(qr2/ea):0.0;
            if(ea>LOCK_MIN_ENV)
                g_lock_level=LOCK_AVERAGE*g_lock_level+(1.0-LOCK_AVERAGE)*fabs(lmr);
            else
                g_lock_level*=LOCK_AVERAGE;
            double lpr_f=fir(g_lpf,IQ_LPF_TAPS,g_zlpr,lpr);
            double lmr_f=fir(g_lpf,IQ_LPF_TAPS,g_zlmr,lmr);
            left =(lpr_f+lmr_f)*gain;
            right=(lpr_f-lmr_f)*gain;
        }else{
            /* Kahn: LSB=Left, USB=Right */
            double lsb=(ia-qa)*0.5;
            double usb=(ia+qa)*0.5;
            double l=fir(g_lpf,IQ_LPF_TAPS,g_zl,lsb)*gain;
            double r=fir(g_lpf,IQ_LPF_TAPS,g_zr,usb)*gain;
            if(antifade){
                /* Smooth L and R power over ~50ms at 96kHz = 4800 samples, coeff = 1-1/4800 */
                static double pwr_l=0, pwr_r=0;
                pwr_l = 0.999*pwr_l + 0.001*(l*l);
                pwr_r = 0.999*pwr_r + 0.001*(r*r);
                double total = pwr_l + pwr_r;
                double blend;
                if(total < 1.0){
                    blend = 1.0;
                } else {
                    double balance = 2.0 * sqrt(pwr_l * pwr_r) / total;
                    blend = balance;
                }
                /* Attack ~10ms, decay ~3s at 96kHz */
                if(blend < g_antifade_blend)
                    g_antifade_blend = 0.985*g_antifade_blend + 0.015*blend;
                else
                    g_antifade_blend = 0.99997*g_antifade_blend + 0.00003*blend;
                double b3 = g_antifade_blend*g_antifade_blend*g_antifade_blend;
                /* Fallback: use AM envelope as mono source - always present */
                double env_raw = ea; /* AGC-scaled envelope */
                double af_y = env_raw - g_af_dc_x1 + DC_R*g_af_dc_y1;
                g_af_dc_x1 = env_raw; g_af_dc_y1 = af_y;
                double env_mono = af_y * gain;
                double m = env_mono;
                left  = m + (l - m)*b3;
                right = m + (r - m)*b3;
            }else{
                left=l; right=r;
                g_antifade_blend=1.0;
            }
        }

        pacc+=1.0;
        if(pacc>=ratio){
            pacc-=ratio;
            left =soft_clip(left);
            right=soft_clip(right);
            if(g_mute_samples>0){left=0;right=0;}
            if(fmono){double m=(left+right)*0.5;left=m;right=m;}
            if(g_nf>0){
                left =ntick(&g_nl1,g_nb0,g_nb1,g_nb2,g_na1,g_na2,left);
                left =ntick(&g_nl2,g_nb0,g_nb1,g_nb2,g_na1,g_na2,left);
                right=ntick(&g_nr1,g_nb0,g_nb1,g_nb2,g_na1,g_na2,right);
                right=ntick(&g_nr2,g_nb0,g_nb1,g_nb2,g_na1,g_na2,right);
            }
            if(g_vf>0){
                left =ntick(&g_vl1,g_vb0,g_vb1,g_vb2,g_va1,g_va2,left);
                left =ntick(&g_vl2,g_vb0,g_vb1,g_vb2,g_va1,g_va2,left);
                right=ntick(&g_vr1,g_vb0,g_vb1,g_vb2,g_va1,g_va2,right);
                right=ntick(&g_vr2,g_vb0,g_vb1,g_vb2,g_va1,g_va2,right);
            }
            if(fabs(left) >pl) pl=fabs(left);
            if(fabs(right)>pr) pr=fabs(right);
            if(oc+1<(DWORD)OUT_SAMPLES){
                out[oc++]=(int16_t)left;
                out[oc++]=(int16_t)right;
            }
        }
    }

    if(oc>0) submit_out(g_out_idx,oc);
    g_out_idx=(g_out_idx+1)%NUM_BUFFERS;

    EnterCriticalSection(&g_state.cs);
    g_state.level_l=pl/32767.0;
    g_state.level_r=pr/32767.0;
    g_state.stereo_lock=(mode==MODE_CQUAM&&g_lock_level>LOCK_THRESHOLD)?1:0;
    LeaveCriticalSection(&g_state.cs);
}

/* ── Decoder thread ──────────────────────────────────────────────────────── */
static DWORD WINAPI decoder_thread(LPVOID param)
{
    HWAVEIN  hIn =((HANDLE*)param)[0];
    HWAVEOUT hOut=((HANDLE*)param)[1];
    free(param);
    g_hWaveIn=hIn; g_hWaveOut=hOut;

    EnterCriticalSection(&g_state.cs);
    double bw=g_state.bw_hz;
    LeaveCriticalSection(&g_state.cs);

    build_lpf(g_lpf,IQ_LPF_TAPS,bw,(double)g_sample_rate);
    memset(g_zi,0,sizeof(g_zi)); memset(g_zq,0,sizeof(g_zq));
    memset(g_zl,0,sizeof(g_zl)); memset(g_zr,0,sizeof(g_zr));
    memset(g_zlpr,0,sizeof(g_zlpr)); memset(g_zlmr,0,sizeof(g_zlmr));
    g_dc_x1=g_dc_y1=0; g_pll_phase=g_pll_freq=0;
    g_agc_gain=1.0; g_lock_level=0; g_out_idx=0;
    g_antifade_blend=1.0; g_af_dc_x1=g_af_dc_y1=0;
    g_last_agc_gain=1.0; g_mute_samples=0; g_pll_err_avg=1.0;

    for(int i=0;i<NUM_BUFFERS;i++){
        memset(&g_in_hdr[i],0,sizeof(WAVEHDR));
        memset(&g_out_hdr[i],0,sizeof(WAVEHDR));
        g_in_hdr[i].lpData=(LPSTR)g_in_buf[i];
        g_out_hdr[i].lpData=(LPSTR)g_out_buf[i];
        requeue_in(i);
    }

    EnterCriticalSection(&g_state.cs);
    g_state.running=1;
    LeaveCriticalSection(&g_state.cs);

    waveInStart(g_hWaveIn);

    double last_bw=bw,last_nf=-1,last_vf=-1;

    while(1){
        EnterCriticalSection(&g_state.cs);
        int        stop  =g_state.cmd_stop;
        double     gain  =g_state.gain;
        DecodeMode mode  =g_state.mode;
        int        fmono =g_state.force_mono;
        int        afade =g_state.antifade;
        double     cur_bw=g_state.bw_hz;
        double     cur_nf=g_state.notch_hz;
        double     cur_vf=g_state.vnotch_hz;
        LeaveCriticalSection(&g_state.cs);

        if(stop) break;

        if(cur_bw!=last_bw&&cur_bw>0){
            last_bw=cur_bw;
            build_lpf(g_lpf,IQ_LPF_TAPS,cur_bw,(double)g_sample_rate);
            memset(g_zi,0,sizeof(g_zi)); memset(g_zq,0,sizeof(g_zq));
            memset(g_zl,0,sizeof(g_zl)); memset(g_zr,0,sizeof(g_zr));
            memset(g_zlpr,0,sizeof(g_zlpr)); memset(g_zlmr,0,sizeof(g_zlmr));
        }
        if(cur_nf!=last_nf){last_nf=cur_nf;set_notch(cur_nf,(double)AUDIO_RATE);}
        if(cur_vf!=last_vf){last_vf=cur_vf;set_vnotch(cur_vf,(double)AUDIO_RATE);}

        WaitForSingleObject(g_hEventIn,100);
        for(int i=0;i<NUM_BUFFERS;i++){
            if(g_in_hdr[i].dwFlags&WHDR_DONE){
                DWORD stride=(g_fmt==FMT_FLOAT32)?8:4;
                DWORD frm=g_in_hdr[i].dwBytesRecorded/stride;
                if(frm>0) process_block(g_in_buf[i],frm,gain,mode,fmono,afade);
                waveInUnprepareHeader(g_hWaveIn,&g_in_hdr[i],sizeof(WAVEHDR));
                requeue_in(i);
            }
        }
    }

    waveInStop(g_hWaveIn); waveOutReset(g_hWaveOut);
    for(int i=0;i<NUM_BUFFERS;i++){
        waveInUnprepareHeader(g_hWaveIn,&g_in_hdr[i],sizeof(WAVEHDR));
        if(g_out_hdr[i].dwFlags&WHDR_PREPARED)
            waveOutUnprepareHeader(g_hWaveOut,&g_out_hdr[i],sizeof(WAVEHDR));
    }
    waveInClose(g_hWaveIn);g_hWaveIn=NULL;
    waveOutClose(g_hWaveOut);g_hWaveOut=NULL;
    CloseHandle(g_hEventIn);CloseHandle(g_hEventOut);

    EnterCriticalSection(&g_state.cs);
    g_state.running=0;g_state.cmd_stop=0;
    g_state.level_l=g_state.level_r=0;g_state.stereo_lock=0;
    LeaveCriticalSection(&g_state.cs);
    return 0;
}

/* ── GUI globals ─────────────────────────────────────────────────────────── */
static HWND g_hWnd;
static HWND g_hDevIn,g_hDevOut;
static HWND g_hModeCQUAM,g_hModeKahn;
static HWND g_hNotchOff,g_hNotch9K,g_hNotch10K;
static HWND g_hBwDX,g_hBwNarrow,g_hBwNormal,g_hBwWide,g_hBwCustom,g_hBwHz;
static HWND g_hGainSlider,g_hGainLabel;
static HWND g_hVNotchChk,g_hVNotchSld,g_hVNotchLabel;
static HWND g_hBtnStart,g_hBtnStop,g_hBtnMono,g_hBtnOnTop,g_hBtnAntifade;
static HWND g_hMeterL,g_hMeterR,g_hStereoLed;
static HANDLE g_hDecoderThread=NULL;
static double g_peak_l=0,g_peak_r=0;
#define METER_DECAY 0.85
static char g_ini_path[MAX_PATH];

/* ── Helpers ─────────────────────────────────────────────────────────────── */
static void get_ini_path(void)
{
    GetModuleFileName(NULL,g_ini_path,MAX_PATH);
    char *dot=strrchr(g_ini_path,'.');
    if(dot) strcpy(dot,".ini"); else strcat(g_ini_path,".ini");
}

static void populate_devices(void)
{
    UINT n=waveInGetNumDevs();
    for(UINT i=0;i<n;i++){
        WAVEINCAPS c;
        if(waveInGetDevCaps(i,&c,sizeof(c))==MMSYSERR_NOERROR)
            SendMessage(g_hDevIn,CB_ADDSTRING,0,(LPARAM)c.szPname);
    }
    SendMessage(g_hDevIn,CB_SETCURSEL,0,0);
    n=waveOutGetNumDevs();
    for(UINT i=0;i<n;i++){
        WAVEOUTCAPS c;
        if(waveOutGetDevCaps(i,&c,sizeof(c))==MMSYSERR_NOERROR)
            SendMessage(g_hDevOut,CB_ADDSTRING,0,(LPARAM)c.szPname);
    }
    SendMessage(g_hDevOut,CB_SETCURSEL,0,0);
}

static double read_bw(void)
{
    if(SendMessage(g_hBwDX,    BM_GETCHECK,0,0)==BST_CHECKED) return 3000.0;
    if(SendMessage(g_hBwNarrow,BM_GETCHECK,0,0)==BST_CHECKED) return 5000.0;
    if(SendMessage(g_hBwNormal,BM_GETCHECK,0,0)==BST_CHECKED) return 9000.0;
    if(SendMessage(g_hBwWide,  BM_GETCHECK,0,0)==BST_CHECKED) return 15000.0;
    char buf[32]; GetWindowText(g_hBwHz,buf,sizeof(buf));
    double v=atof(buf); return(v>0)?v:9000.0;
}

static void update_gain_label(void)
{
    int pos=(int)SendMessage(g_hGainSlider,TBM_GETPOS,0,0);
    double g=GAIN_MIN+(GAIN_MAX-GAIN_MIN)*(pos-GAIN_SLIDER_MIN)
             /(double)(GAIN_SLIDER_MAX-GAIN_SLIDER_MIN);
    char buf[32]; snprintf(buf,sizeof(buf),"Gain: %.1f",g);
    SetWindowText(g_hGainLabel,buf);
    EnterCriticalSection(&g_state.cs);
    g_state.gain=g;
    LeaveCriticalSection(&g_state.cs);
}

/* ── Drawing ─────────────────────────────────────────────────────────────── */
static void draw_meter(HDC hdc,HWND hm,double level)
{
    RECT rc; GetClientRect(hm,&rc);
    int w=rc.right-rc.left,fill=(int)(level*w);
    if(fill>w) fill=w;
    HBRUSH bg=CreateSolidBrush(RGB(30,30,30));
    FillRect(hdc,&rc,bg); DeleteObject(bg);
    if(fill>0){
        int ys=(int)(w*0.7),rs=(int)(w*0.9);
        RECT seg=rc;
        seg.right=rc.left+((fill<ys)?fill:ys);
        HBRUSH br=CreateSolidBrush(RGB(0,200,0)); FillRect(hdc,&seg,br); DeleteObject(br);
        if(fill>ys){
            seg.left=rc.left+ys; seg.right=rc.left+((fill<rs)?fill:rs);
            br=CreateSolidBrush(RGB(220,200,0)); FillRect(hdc,&seg,br); DeleteObject(br);
        }
        if(fill>rs){
            seg.left=rc.left+rs; seg.right=rc.left+fill;
            br=CreateSolidBrush(RGB(220,0,0)); FillRect(hdc,&seg,br); DeleteObject(br);
        }
    }
    FrameRect(hdc,&rc,(HBRUSH)GetStockObject(GRAY_BRUSH));
}

static void draw_led(HDC hdc,HWND hl,int on)
{
    RECT rc; GetClientRect(hl,&rc);
    HBRUSH br=CreateSolidBrush(on?RGB(220,0,0):RGB(60,0,0));
    HPEN pe=CreatePen(PS_SOLID,1,RGB(80,80,80));
    HBRUSH ob=SelectObject(hdc,br); HPEN op=SelectObject(hdc,pe);
    Rectangle(hdc,rc.left,rc.top,rc.right,rc.bottom);
    SelectObject(hdc,ob); SelectObject(hdc,op);
    DeleteObject(br); DeleteObject(pe);
}

/* ── Probe and open ──────────────────────────────────────────────────────── */
static MMRESULT probe_and_open_in(UINT dev,DWORD *rate,InputFmt *fmt)
{
    DWORD rates[]={96000,192000,48000,0};
    for(int ri=0;rates[ri];ri++){
        InputFmt fmts[2]={FMT_FLOAT32,FMT_INT16};
        for(int fi=0;fi<2;fi++){
            WAVEFORMATEX wfx={0};
            wfx.nChannels=CHANNELS_IQ; wfx.nSamplesPerSec=rates[ri];
            if(fmts[fi]==FMT_FLOAT32){wfx.wFormatTag=WAVE_FORMAT_IEEE_FLOAT;wfx.wBitsPerSample=32;}
            else{wfx.wFormatTag=WAVE_FORMAT_PCM;wfx.wBitsPerSample=16;}
            wfx.nBlockAlign=(wfx.nChannels*wfx.wBitsPerSample)/8;
            wfx.nAvgBytesPerSec=rates[ri]*wfx.nBlockAlign;
            if(waveInOpen(NULL,dev,&wfx,0,0,WAVE_FORMAT_QUERY)!=MMSYSERR_NOERROR) continue;
            MMRESULT r=waveInOpen(&g_hWaveIn,dev,&wfx,(DWORD_PTR)g_hEventIn,0,CALLBACK_EVENT);
            if(r==MMSYSERR_NOERROR){*rate=rates[ri];*fmt=fmts[fi];return MMSYSERR_NOERROR;}
        }
    }
    return MMSYSERR_ERROR;
}

static void start_decoder(void)
{
    UINT in_dev =(UINT)SendMessage(g_hDevIn, CB_GETCURSEL,0,0);
    UINT out_dev=(UINT)SendMessage(g_hDevOut,CB_GETCURSEL,0,0);
    g_hEventIn =CreateEvent(NULL,FALSE,FALSE,NULL);
    g_hEventOut=CreateEvent(NULL,FALSE,FALSE,NULL);
    DWORD rate=0; InputFmt fmt=FMT_FLOAT32;
    if(probe_and_open_in(in_dev,&rate,&fmt)!=MMSYSERR_NOERROR){
        CloseHandle(g_hEventIn); CloseHandle(g_hEventOut);
        MessageBox(g_hWnd,
            "Could not open input device.\n"
            "Select the VAC (CABLE Output) and ensure SDRuno IQ OUT is active.",
            "AM Stereo Decoder",MB_OK|MB_ICONWARNING);
        return;
    }
    g_sample_rate=rate; g_fmt=fmt;
    WAVEFORMATEX wfx={0};
    wfx.wFormatTag=WAVE_FORMAT_PCM; wfx.nChannels=AUDIO_CHANNELS;
    wfx.nSamplesPerSec=AUDIO_RATE; wfx.wBitsPerSample=AUDIO_BITS;
    wfx.nBlockAlign=AUDIO_CHANNELS*(AUDIO_BITS/8);
    wfx.nAvgBytesPerSec=AUDIO_RATE*wfx.nBlockAlign;
    MMRESULT r=waveOutOpen(&g_hWaveOut,out_dev,&wfx,(DWORD_PTR)g_hEventOut,0,CALLBACK_EVENT);
    if(r!=MMSYSERR_NOERROR){
        waveInClose(g_hWaveIn); g_hWaveIn=NULL;
        CloseHandle(g_hEventIn); CloseHandle(g_hEventOut);
        MessageBox(g_hWnd,"Could not open output device.","AM Stereo Decoder",MB_OK|MB_ICONWARNING);
        return;
    }
    EnterCriticalSection(&g_state.cs);
    g_state.bw_hz    =read_bw();
    g_state.gain     =GAIN_MIN+(GAIN_MAX-GAIN_MIN)
                      *((int)SendMessage(g_hGainSlider,TBM_GETPOS,0,0)-GAIN_SLIDER_MIN)
                      /(double)(GAIN_SLIDER_MAX-GAIN_SLIDER_MIN);
    g_state.mode     =(SendMessage(g_hModeKahn,BM_GETCHECK,0,0)==BST_CHECKED)?MODE_KAHN:MODE_CQUAM;
    g_state.force_mono=(SendMessage(g_hBtnMono,BM_GETCHECK,0,0)==BST_CHECKED)?1:0;
    g_state.notch_hz =(SendMessage(g_hNotch9K, BM_GETCHECK,0,0)==BST_CHECKED)?9000.0:
                      (SendMessage(g_hNotch10K,BM_GETCHECK,0,0)==BST_CHECKED)?10000.0:0.0;
    g_state.vnotch_hz=(SendMessage(g_hVNotchChk,BM_GETCHECK,0,0)==BST_CHECKED)?
                      (double)(SendMessage(g_hVNotchSld,TBM_GETPOS,0,0)*100):0.0;
    g_state.cmd_stop =0;
    LeaveCriticalSection(&g_state.cs);
    HANDLE *h=malloc(2*sizeof(HANDLE)); h[0]=g_hWaveIn; h[1]=g_hWaveOut;
    if(g_hDecoderThread){CloseHandle(g_hDecoderThread);g_hDecoderThread=NULL;}
    g_hDecoderThread=CreateThread(NULL,0,decoder_thread,h,0,NULL);
}

/* ── Save/Load settings ──────────────────────────────────────────────────── */
static void save_settings(void)
{
    char buf[32];
    snprintf(buf,sizeof(buf),"%d",(int)SendMessage(g_hDevIn, CB_GETCURSEL,0,0));
    WritePrivateProfileString("Settings","DevIn", buf,g_ini_path);
    snprintf(buf,sizeof(buf),"%d",(int)SendMessage(g_hDevOut,CB_GETCURSEL,0,0));
    WritePrivateProfileString("Settings","DevOut",buf,g_ini_path);
    WritePrivateProfileString("Settings","Mode",
        (SendMessage(g_hModeKahn,BM_GETCHECK,0,0)==BST_CHECKED)?"kahn":"cquam",g_ini_path);
    const char *bw="normal";
    if(SendMessage(g_hBwDX,    BM_GETCHECK,0,0)==BST_CHECKED) bw="dx";
    if(SendMessage(g_hBwNarrow,BM_GETCHECK,0,0)==BST_CHECKED) bw="narrow";
    if(SendMessage(g_hBwWide,  BM_GETCHECK,0,0)==BST_CHECKED) bw="wide";
    if(SendMessage(g_hBwCustom,BM_GETCHECK,0,0)==BST_CHECKED) bw="custom";
    WritePrivateProfileString("Settings","Bandwidth",bw,g_ini_path);
    GetWindowText(g_hBwHz,buf,sizeof(buf));
    WritePrivateProfileString("Settings","CustomHz",buf,g_ini_path);
    snprintf(buf,sizeof(buf),"%d",(int)SendMessage(g_hGainSlider,TBM_GETPOS,0,0));
    WritePrivateProfileString("Settings","GainPos",buf,g_ini_path);
    const char *notch="off";
    if(SendMessage(g_hNotch9K, BM_GETCHECK,0,0)==BST_CHECKED) notch="9k";
    if(SendMessage(g_hNotch10K,BM_GETCHECK,0,0)==BST_CHECKED) notch="10k";
    WritePrivateProfileString("Settings","Notch",notch,g_ini_path);
    WritePrivateProfileString("Settings","Mono",
        (SendMessage(g_hBtnMono,    BM_GETCHECK,0,0)==BST_CHECKED)?"1":"0",g_ini_path);
    WritePrivateProfileString("Settings","Antifade",
        (SendMessage(g_hBtnAntifade,BM_GETCHECK,0,0)==BST_CHECKED)?"1":"0",g_ini_path);
    WritePrivateProfileString("Settings","OnTop",
        (SendMessage(g_hBtnOnTop,BM_GETCHECK,0,0)==BST_CHECKED)?"1":"0",g_ini_path);
    snprintf(buf,sizeof(buf),"%d",(int)SendMessage(g_hVNotchSld,TBM_GETPOS,0,0));
    WritePrivateProfileString("Settings","VNotchPos",buf,g_ini_path);
    WritePrivateProfileString("Settings","VNotchOn",
        (SendMessage(g_hVNotchChk,BM_GETCHECK,0,0)==BST_CHECKED)?"1":"0",g_ini_path);
}

static void load_settings(void)
{
    char buf[64];
    int dev=GetPrivateProfileInt("Settings","DevIn",0,g_ini_path);
    if(dev<(int)SendMessage(g_hDevIn,CB_GETCOUNT,0,0)) SendMessage(g_hDevIn,CB_SETCURSEL,dev,0);
    dev=GetPrivateProfileInt("Settings","DevOut",0,g_ini_path);
    if(dev<(int)SendMessage(g_hDevOut,CB_GETCOUNT,0,0)) SendMessage(g_hDevOut,CB_SETCURSEL,dev,0);
    GetPrivateProfileString("Settings","Mode","cquam",buf,sizeof(buf),g_ini_path);
    if(strcmp(buf,"kahn")==0){
        SendMessage(g_hModeCQUAM,BM_SETCHECK,BST_UNCHECKED,0);
        SendMessage(g_hModeKahn, BM_SETCHECK,BST_CHECKED,  0);
    }
    GetPrivateProfileString("Settings","Bandwidth","normal",buf,sizeof(buf),g_ini_path);
    SendMessage(g_hBwDX,    BM_SETCHECK,BST_UNCHECKED,0);
    SendMessage(g_hBwNarrow,BM_SETCHECK,BST_UNCHECKED,0);
    SendMessage(g_hBwNormal,BM_SETCHECK,BST_UNCHECKED,0);
    SendMessage(g_hBwWide,  BM_SETCHECK,BST_UNCHECKED,0);
    SendMessage(g_hBwCustom,BM_SETCHECK,BST_UNCHECKED,0);
    if     (strcmp(buf,"dx"    )==0) SendMessage(g_hBwDX,    BM_SETCHECK,BST_CHECKED,0);
    else if(strcmp(buf,"narrow")==0) SendMessage(g_hBwNarrow,BM_SETCHECK,BST_CHECKED,0);
    else if(strcmp(buf,"wide"  )==0) SendMessage(g_hBwWide,  BM_SETCHECK,BST_CHECKED,0);
    else if(strcmp(buf,"custom")==0) SendMessage(g_hBwCustom,BM_SETCHECK,BST_CHECKED,0);
    else                             SendMessage(g_hBwNormal,BM_SETCHECK,BST_CHECKED,0);
    GetPrivateProfileString("Settings","CustomHz","9000",buf,sizeof(buf),g_ini_path);
    SetWindowText(g_hBwHz,buf);
    int gpos=GetPrivateProfileInt("Settings","GainPos",
        (int)(GAIN_SLIDER_MIN+(2.0-GAIN_MIN)/(GAIN_MAX-GAIN_MIN)*(GAIN_SLIDER_MAX-GAIN_SLIDER_MIN)),
        g_ini_path);
    SendMessage(g_hGainSlider,TBM_SETPOS,TRUE,gpos); update_gain_label();
    GetPrivateProfileString("Settings","Notch","off",buf,sizeof(buf),g_ini_path);
    SendMessage(g_hNotchOff,BM_SETCHECK,BST_UNCHECKED,0);
    SendMessage(g_hNotch9K, BM_SETCHECK,BST_UNCHECKED,0);
    SendMessage(g_hNotch10K,BM_SETCHECK,BST_UNCHECKED,0);
    if     (strcmp(buf,"9k" )==0) SendMessage(g_hNotch9K, BM_SETCHECK,BST_CHECKED,0);
    else if(strcmp(buf,"10k")==0) SendMessage(g_hNotch10K,BM_SETCHECK,BST_CHECKED,0);
    else                          SendMessage(g_hNotchOff,BM_SETCHECK,BST_CHECKED,0);
    if(GetPrivateProfileInt("Settings","Mono",0,g_ini_path))
        SendMessage(g_hBtnMono,BM_SETCHECK,BST_CHECKED,0);
    if(GetPrivateProfileInt("Settings","Antifade",0,g_ini_path))
        SendMessage(g_hBtnAntifade,BM_SETCHECK,BST_CHECKED,0);
    if(GetPrivateProfileInt("Settings","OnTop",0,g_ini_path)){
        SendMessage(g_hBtnOnTop,BM_SETCHECK,BST_CHECKED,0);
        SetWindowPos(g_hWnd,HWND_TOPMOST,0,0,0,0,SWP_NOMOVE|SWP_NOSIZE);
    }
    int vnpos=GetPrivateProfileInt("Settings","VNotchPos",10,g_ini_path);
    SendMessage(g_hVNotchSld,TBM_SETPOS,TRUE,vnpos);
    char vnbuf[16]; snprintf(vnbuf,sizeof(vnbuf),"%.1f kHz",vnpos*0.1);
    SetWindowText(g_hVNotchLabel,vnbuf);
    if(GetPrivateProfileInt("Settings","VNotchOn",0,g_ini_path)){
        SendMessage(g_hVNotchChk,BM_SETCHECK,BST_CHECKED,0);
        EnableWindow(g_hVNotchSld,TRUE);
    }
}

/* ── Window procedure ────────────────────────────────────────────────────── */
static LRESULT CALLBACK WndProc(HWND hWnd,UINT msg,WPARAM wParam,LPARAM lParam)
{
    switch(msg){
    case WM_CREATE:{
        int x=10,y=10,w=560;
        HFONT hf=(HFONT)GetStockObject(DEFAULT_GUI_FONT);
        HWND h;

        /* Banner */
        CreateWindow("STATIC",NULL,WS_CHILD|WS_VISIBLE|SS_OWNERDRAW,
            0,0,600,36,hWnd,(HMENU)200,NULL,NULL);
        y=46;

        /* Devices */
        h=CreateWindow("STATIC","In:",WS_CHILD|WS_VISIBLE,x,y+3,20,20,hWnd,NULL,NULL,NULL);
        SendMessage(h,WM_SETFONT,(WPARAM)hf,TRUE);
        g_hDevIn=CreateWindow("COMBOBOX",NULL,WS_CHILD|WS_VISIBLE|WS_VSCROLL|CBS_DROPDOWNLIST,
            x+22,y,200,200,hWnd,(HMENU)IDC_DEV_IN,NULL,NULL);
        SendMessage(g_hDevIn,WM_SETFONT,(WPARAM)hf,TRUE);
        h=CreateWindow("STATIC","Out:",WS_CHILD|WS_VISIBLE,x+235,y+3,28,20,hWnd,NULL,NULL,NULL);
        SendMessage(h,WM_SETFONT,(WPARAM)hf,TRUE);
        g_hDevOut=CreateWindow("COMBOBOX",NULL,WS_CHILD|WS_VISIBLE|WS_VSCROLL|CBS_DROPDOWNLIST,
            x+265,y,260,200,hWnd,(HMENU)IDC_DEV_OUT,NULL,NULL);
        SendMessage(g_hDevOut,WM_SETFONT,(WPARAM)hf,TRUE);
        y+=35;

        /* Mode + Whistle filter */
        h=CreateWindow("STATIC","Mode:",WS_CHILD|WS_VISIBLE,x,y+3,75,20,hWnd,NULL,NULL,NULL);
        SendMessage(h,WM_SETFONT,(WPARAM)hf,TRUE);
        g_hModeCQUAM=CreateWindow("BUTTON","C-QUAM",WS_CHILD|WS_VISIBLE|BS_AUTORADIOBUTTON|WS_GROUP,
            x+78,y,80,20,hWnd,(HMENU)IDC_MODE_CQUAM,NULL,NULL);
        SendMessage(g_hModeCQUAM,WM_SETFONT,(WPARAM)hf,TRUE);
        SendMessage(g_hModeCQUAM,BM_SETCHECK,BST_CHECKED,0);
        g_hModeKahn=CreateWindow("BUTTON","Kahn",WS_CHILD|WS_VISIBLE|BS_AUTORADIOBUTTON,
            x+163,y,55,20,hWnd,(HMENU)IDC_MODE_KAHN,NULL,NULL);
        SendMessage(g_hModeKahn,WM_SETFONT,(WPARAM)hf,TRUE);
        h=CreateWindow("STATIC","Whistle filter:",WS_CHILD|WS_VISIBLE,x+230,y+3,90,20,hWnd,NULL,NULL,NULL);
        SendMessage(h,WM_SETFONT,(WPARAM)hf,TRUE);
        g_hNotchOff=CreateWindow("BUTTON","Off",WS_CHILD|WS_VISIBLE|BS_AUTORADIOBUTTON|WS_GROUP,
            x+323,y,38,20,hWnd,(HMENU)IDC_NOTCH_OFF,NULL,NULL);
        SendMessage(g_hNotchOff,WM_SETFONT,(WPARAM)hf,TRUE);
        SendMessage(g_hNotchOff,BM_SETCHECK,BST_CHECKED,0);
        g_hNotch9K=CreateWindow("BUTTON","9k",WS_CHILD|WS_VISIBLE|BS_AUTORADIOBUTTON,
            x+364,y,38,20,hWnd,(HMENU)IDC_NOTCH_9K,NULL,NULL);
        SendMessage(g_hNotch9K,WM_SETFONT,(WPARAM)hf,TRUE);
        g_hNotch10K=CreateWindow("BUTTON","10k",WS_CHILD|WS_VISIBLE|BS_AUTORADIOBUTTON,
            x+405,y,42,20,hWnd,(HMENU)IDC_NOTCH_10K,NULL,NULL);
        SendMessage(g_hNotch10K,WM_SETFONT,(WPARAM)hf,TRUE);
        y+=28;

        /* Bandwidth */
        h=CreateWindow("STATIC","Bandwidth:",WS_CHILD|WS_VISIBLE,x,y+3,75,20,hWnd,NULL,NULL,NULL);
        SendMessage(h,WM_SETFONT,(WPARAM)hf,TRUE);
        int bx=x+78;
        g_hBwDX=CreateWindow("BUTTON","DX",WS_CHILD|WS_VISIBLE|BS_AUTORADIOBUTTON|WS_GROUP,
            bx,y,40,20,hWnd,(HMENU)IDC_BW_DX,NULL,NULL);
        SendMessage(g_hBwDX,WM_SETFONT,(WPARAM)hf,TRUE); bx+=43;
        g_hBwNarrow=CreateWindow("BUTTON","Narrow",WS_CHILD|WS_VISIBLE|BS_AUTORADIOBUTTON,
            bx,y,62,20,hWnd,(HMENU)IDC_BW_NARROW,NULL,NULL);
        SendMessage(g_hBwNarrow,WM_SETFONT,(WPARAM)hf,TRUE); bx+=65;
        g_hBwNormal=CreateWindow("BUTTON","Normal",WS_CHILD|WS_VISIBLE|BS_AUTORADIOBUTTON,
            bx,y,62,20,hWnd,(HMENU)IDC_BW_NORMAL,NULL,NULL);
        SendMessage(g_hBwNormal,WM_SETFONT,(WPARAM)hf,TRUE);
        SendMessage(g_hBwNormal,BM_SETCHECK,BST_CHECKED,0); bx+=65;
        g_hBwWide=CreateWindow("BUTTON","Wide",WS_CHILD|WS_VISIBLE|BS_AUTORADIOBUTTON,
            bx,y,50,20,hWnd,(HMENU)IDC_BW_WIDE,NULL,NULL);
        SendMessage(g_hBwWide,WM_SETFONT,(WPARAM)hf,TRUE); bx+=53;
        g_hBwCustom=CreateWindow("BUTTON","Custom:",WS_CHILD|WS_VISIBLE|BS_AUTORADIOBUTTON,
            bx,y,68,20,hWnd,(HMENU)IDC_BW_CUSTOM,NULL,NULL);
        SendMessage(g_hBwCustom,WM_SETFONT,(WPARAM)hf,TRUE); bx+=71;
        g_hBwHz=CreateWindow("EDIT","9000",WS_CHILD|WS_VISIBLE|WS_BORDER|ES_NUMBER,
            bx,y,48,20,hWnd,(HMENU)IDC_BW_HZ,NULL,NULL);
        SendMessage(g_hBwHz,WM_SETFONT,(WPARAM)hf,TRUE); bx+=51;
        h=CreateWindow("STATIC","Hz",WS_CHILD|WS_VISIBLE,bx,y+3,20,20,hWnd,NULL,NULL,NULL);
        SendMessage(h,WM_SETFONT,(WPARAM)hf,TRUE);
        y+=28;

        /* Gain */
        g_hGainLabel=CreateWindow("STATIC","Gain: 2.0",WS_CHILD|WS_VISIBLE,
            x,y+5,70,20,hWnd,(HMENU)IDC_GAIN_LABEL,NULL,NULL);
        SendMessage(g_hGainLabel,WM_SETFONT,(WPARAM)hf,TRUE);
        g_hGainSlider=CreateWindow(TRACKBAR_CLASS,NULL,WS_CHILD|WS_VISIBLE|TBS_HORZ|TBS_NOTICKS,
            x+75,y,w-80,26,hWnd,(HMENU)IDC_GAIN_SLIDER,NULL,NULL);
        SendMessage(g_hGainSlider,TBM_SETRANGE,TRUE,MAKELPARAM(GAIN_SLIDER_MIN,GAIN_SLIDER_MAX));
        SendMessage(g_hGainSlider,TBM_SETPOS,TRUE,
            (int)(GAIN_SLIDER_MIN+(2.0-GAIN_MIN)/(GAIN_MAX-GAIN_MIN)*(GAIN_SLIDER_MAX-GAIN_SLIDER_MIN)));
        y+=36;

        /* Het notch */
        g_hVNotchChk=CreateWindow("BUTTON","Het Notch",WS_CHILD|WS_VISIBLE|BS_AUTOCHECKBOX,
            x,y+1,82,22,hWnd,(HMENU)IDC_VNOTCH_CHK,NULL,NULL);
        SendMessage(g_hVNotchChk,WM_SETFONT,(WPARAM)hf,TRUE);
        g_hVNotchSld=CreateWindow(TRACKBAR_CLASS,NULL,
            WS_CHILD|WS_VISIBLE|TBS_HORZ|TBS_NOTICKS|TBS_AUTOTICKS,
            x+90,y,w-175,26,hWnd,(HMENU)IDC_VNOTCH_SLD,NULL,NULL);
        SendMessage(g_hVNotchSld,TBM_SETRANGE,TRUE,MAKELPARAM(5,85));
        SendMessage(g_hVNotchSld,TBM_SETPOS,TRUE,10);
        SendMessage(g_hVNotchSld,TBM_SETTICFREQ,10,0);
        g_hVNotchLabel=CreateWindow("STATIC","1.0 kHz",WS_CHILD|WS_VISIBLE,
            x+w-80,y+5,65,18,hWnd,(HMENU)IDC_VNOTCH_LABEL,NULL,NULL);
        SendMessage(g_hVNotchLabel,WM_SETFONT,(WPARAM)hf,TRUE);
        EnableWindow(g_hVNotchSld,FALSE);
        y+=36;

        /* Meters */
        h=CreateWindow("STATIC","L",WS_CHILD|WS_VISIBLE,x,y+4,12,18,hWnd,NULL,NULL,NULL);
        SendMessage(h,WM_SETFONT,(WPARAM)hf,TRUE);
        g_hMeterL=CreateWindow("STATIC",NULL,WS_CHILD|WS_VISIBLE|SS_OWNERDRAW,
            x+16,y,w-20,18,hWnd,(HMENU)IDC_METER_L,NULL,NULL);
        y+=24;
        h=CreateWindow("STATIC","R",WS_CHILD|WS_VISIBLE,x,y+4,12,18,hWnd,NULL,NULL,NULL);
        SendMessage(h,WM_SETFONT,(WPARAM)hf,TRUE);
        g_hMeterR=CreateWindow("STATIC",NULL,WS_CHILD|WS_VISIBLE|SS_OWNERDRAW,
            x+16,y,w-20,18,hWnd,(HMENU)IDC_METER_R,NULL,NULL);
        y+=34;

        /* Bottom row */
        g_hStereoLed=CreateWindow("STATIC",NULL,WS_CHILD|WS_VISIBLE|SS_OWNERDRAW,
            x,y+2,16,18,hWnd,(HMENU)IDC_STEREO_LED,NULL,NULL);
        h=CreateWindow("STATIC","Stereo",WS_CHILD|WS_VISIBLE,x+20,y+2,42,18,hWnd,NULL,NULL,NULL);
        SendMessage(h,WM_SETFONT,(WPARAM)hf,TRUE);
        g_hBtnMono=CreateWindow("BUTTON","Mono",WS_CHILD|WS_VISIBLE|BS_AUTOCHECKBOX,
            x+70,y+1,55,22,hWnd,(HMENU)IDC_BTN_MONO,NULL,NULL);
        SendMessage(g_hBtnMono,WM_SETFONT,(WPARAM)hf,TRUE);
        g_hBtnOnTop=CreateWindow("BUTTON","On Top",WS_CHILD|WS_VISIBLE|BS_AUTOCHECKBOX,
            x+130,y+1,65,22,hWnd,(HMENU)IDC_BTN_ONTOP,NULL,NULL);
        SendMessage(g_hBtnOnTop,WM_SETFONT,(WPARAM)hf,TRUE);
        g_hBtnAntifade=CreateWindow("BUTTON","Anti-fade",WS_CHILD|WS_VISIBLE|BS_AUTOCHECKBOX,
            x+200,y+1,75,22,hWnd,(HMENU)IDC_BTN_ANTIFADE,NULL,NULL);
        SendMessage(g_hBtnAntifade,WM_SETFONT,(WPARAM)hf,TRUE);
        g_hBtnStart=CreateWindow("BUTTON","Start",WS_CHILD|WS_VISIBLE|BS_PUSHBUTTON,
            x+310,y,80,24,hWnd,(HMENU)IDC_BTN_START,NULL,NULL);
        SendMessage(g_hBtnStart,WM_SETFONT,(WPARAM)hf,TRUE);
        g_hBtnStop=CreateWindow("BUTTON","Stop",WS_CHILD|WS_VISIBLE|BS_PUSHBUTTON,
            x+400,y,80,24,hWnd,(HMENU)IDC_BTN_STOP,NULL,NULL);
        SendMessage(g_hBtnStop,WM_SETFONT,(WPARAM)hf,TRUE);
        EnableWindow(g_hBtnStop,FALSE);

        populate_devices();
        load_settings();
        SetTimer(hWnd,IDC_TIMER,50,NULL);
        break;}

    case WM_TIMER:{
        EnterCriticalSection(&g_state.cs);
        double ll=g_state.level_l,lr=g_state.level_r;
        int slk=g_state.stereo_lock,run=g_state.running;
        LeaveCriticalSection(&g_state.cs);

        if(ll>g_peak_l) g_peak_l=ll; else g_peak_l*=METER_DECAY;
        if(lr>g_peak_r) g_peak_r=lr; else g_peak_r*=METER_DECAY;

        HDC hdc=GetDC(g_hMeterL); draw_meter(hdc,g_hMeterL,g_peak_l); ReleaseDC(g_hMeterL,hdc);
        hdc=GetDC(g_hMeterR);     draw_meter(hdc,g_hMeterR,g_peak_r); ReleaseDC(g_hMeterR,hdc);
        hdc=GetDC(g_hStereoLed);  draw_led(hdc,g_hStereoLed,slk);     ReleaseDC(g_hStereoLed,hdc);

        EnableWindow(g_hBtnStart,!run);
        EnableWindow(g_hBtnStop,  run);

        if(run){
            EnterCriticalSection(&g_state.cs);
            g_state.bw_hz    =read_bw();
            g_state.mode     =(SendMessage(g_hModeKahn,BM_GETCHECK,0,0)==BST_CHECKED)?MODE_KAHN:MODE_CQUAM;
            g_state.force_mono=(SendMessage(g_hBtnMono,BM_GETCHECK,0,0)==BST_CHECKED)?1:0;
            g_state.antifade  =(SendMessage(g_hBtnAntifade,BM_GETCHECK,0,0)==BST_CHECKED)?1:0;
            g_state.notch_hz =(SendMessage(g_hNotch9K, BM_GETCHECK,0,0)==BST_CHECKED)?9000.0:
                              (SendMessage(g_hNotch10K,BM_GETCHECK,0,0)==BST_CHECKED)?10000.0:0.0;
            g_state.vnotch_hz=(SendMessage(g_hVNotchChk,BM_GETCHECK,0,0)==BST_CHECKED)?
                              (double)(SendMessage(g_hVNotchSld,TBM_GETPOS,0,0)*100):0.0;
            LeaveCriticalSection(&g_state.cs);
        }
        break;}

    case WM_HSCROLL:
        if((HWND)lParam==g_hGainSlider) update_gain_label();
        if((HWND)lParam==g_hVNotchSld){
            int pos=(int)SendMessage(g_hVNotchSld,TBM_GETPOS,0,0);
            char buf[16]; snprintf(buf,sizeof(buf),"%.1f kHz",pos*0.1);
            SetWindowText(g_hVNotchLabel,buf);
            if(SendMessage(g_hVNotchChk,BM_GETCHECK,0,0)==BST_CHECKED){
                EnterCriticalSection(&g_state.cs);
                g_state.vnotch_hz=(double)(pos*100);
                LeaveCriticalSection(&g_state.cs);
            }
        }
        break;

    case WM_COMMAND:
        switch(LOWORD(wParam)){
        case IDC_BTN_START:
            if(!g_state.running) start_decoder();
            break;
        case IDC_BTN_STOP:
            EnterCriticalSection(&g_state.cs);
            g_state.cmd_stop=1;
            LeaveCriticalSection(&g_state.cs);
            if(g_hEventIn) SetEvent(g_hEventIn);
            break;
        case IDC_BTN_ONTOP:{
            int on=(SendMessage(g_hBtnOnTop,BM_GETCHECK,0,0)==BST_CHECKED);
            SetWindowPos(hWnd,on?HWND_TOPMOST:HWND_NOTOPMOST,0,0,0,0,SWP_NOMOVE|SWP_NOSIZE);
            break;}
        case IDC_VNOTCH_CHK:{
            int on=(SendMessage(g_hVNotchChk,BM_GETCHECK,0,0)==BST_CHECKED);
            EnableWindow(g_hVNotchSld,on);
            int pos=(int)SendMessage(g_hVNotchSld,TBM_GETPOS,0,0);
            char buf[16]; snprintf(buf,sizeof(buf),"%.1f kHz",pos*0.1);
            SetWindowText(g_hVNotchLabel,buf);
            EnterCriticalSection(&g_state.cs);
            g_state.vnotch_hz=on?(double)(pos*100):0.0;
            LeaveCriticalSection(&g_state.cs);
            break;}
        }
        break;

    case WM_DRAWITEM:{
        DRAWITEMSTRUCT *dis=(DRAWITEMSTRUCT*)lParam;
        if(dis->CtlID==200){
            HBRUSH br=CreateSolidBrush(RGB(25,50,100));
            FillRect(dis->hDC,&dis->rcItem,br); DeleteObject(br);
            SetTextColor(dis->hDC,RGB(255,255,255));
            SetBkMode(dis->hDC,TRANSPARENT);
            HFONT hf=CreateFont(18,0,0,0,FW_BOLD,0,0,0,DEFAULT_CHARSET,
                OUT_DEFAULT_PRECIS,CLIP_DEFAULT_PRECIS,DEFAULT_QUALITY,DEFAULT_PITCH,"Segoe UI");
            HFONT of=SelectObject(dis->hDC,hf);
            /* Title on left */
            RECT tr=dis->rcItem; tr.left+=10;
            DrawText(dis->hDC,"AM Stereo Decoder  v1.0",-1,&tr,DT_LEFT|DT_VCENTER|DT_SINGLELINE);
            /* GitHub URL on right */
            RECT rr=dis->rcItem; rr.right-=10;
            SetTextColor(dis->hDC,RGB(180,210,255));
            DrawText(dis->hDC,"github.com/45south",-1,&rr,DT_RIGHT|DT_VCENTER|DT_SINGLELINE);
            SelectObject(dis->hDC,of); DeleteObject(hf);
            return TRUE;
        }
        if(dis->CtlID==IDC_METER_L) draw_meter(dis->hDC,dis->hwndItem,g_peak_l);
        if(dis->CtlID==IDC_METER_R) draw_meter(dis->hDC,dis->hwndItem,g_peak_r);
        if(dis->CtlID==IDC_STEREO_LED){
            EnterCriticalSection(&g_state.cs);
            int slk=g_state.stereo_lock;
            LeaveCriticalSection(&g_state.cs);
            draw_led(dis->hDC,dis->hwndItem,slk);
        }
        return TRUE;}

    case WM_DESTROY:
        KillTimer(hWnd,IDC_TIMER);
        save_settings();
        EnterCriticalSection(&g_state.cs);
        g_state.cmd_stop=1;
        LeaveCriticalSection(&g_state.cs);
        if(g_hEventIn) SetEvent(g_hEventIn);
        if(g_hDecoderThread){
            WaitForSingleObject(g_hDecoderThread,3000);
            CloseHandle(g_hDecoderThread);
        }
        DeleteCriticalSection(&g_state.cs);
        PostQuitMessage(0);
        break;
    }
    return DefWindowProc(hWnd,msg,wParam,lParam);
}

/* ── WinMain ─────────────────────────────────────────────────────────────── */
int WINAPI WinMain(HINSTANCE hInst,HINSTANCE hPrev,LPSTR lpCmd,int nShow)
{
    (void)hPrev;(void)lpCmd;
    InitCommonControls();
    get_ini_path();
    InitializeCriticalSection(&g_state.cs);
    g_state.bw_hz=9000.0; g_state.gain=2.0;
    g_state.mode=MODE_CQUAM; g_state.force_mono=0; g_state.antifade=0;
    g_state.notch_hz=0; g_state.vnotch_hz=0;

    WNDCLASSEX wc={0};
    wc.cbSize=sizeof(wc); wc.lpfnWndProc=WndProc;
    wc.hInstance=hInst; wc.hbrBackground=(HBRUSH)(COLOR_BTNFACE+1);
    wc.lpszClassName="AMStereoDecoder";
    wc.hIcon=LoadIcon(NULL,IDI_APPLICATION);
    wc.hCursor=LoadCursor(NULL,IDC_ARROW);
    RegisterClassEx(&wc);

    g_hWnd=CreateWindowEx(0,"AMStereoDecoder","AM Stereo Decoder",
        WS_OVERLAPPED|WS_CAPTION|WS_SYSMENU|WS_MINIMIZEBOX,
        CW_USEDEFAULT,CW_USEDEFAULT,610,400,NULL,NULL,hInst,NULL);
    ShowWindow(g_hWnd,nShow);
    UpdateWindow(g_hWnd);

    MSG m;
    while(GetMessage(&m,NULL,0,0)){
        TranslateMessage(&m);
        DispatchMessage(&m);
    }
    return (int)m.wParam;
}
