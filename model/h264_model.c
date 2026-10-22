/*
 * h264_model.c - bit-exact C reference model of the H.264 Constrained
 * Baseline decoder implemented in VHDL under ../rtl.
 *
 * Purpose: golden model for the RTL testbenches.  The structure follows
 * the hardware partitioning (NAL front-end -> syntax decoder -> reconstruction
 * -> deblocking -> frame store) so that per-stage traces can be compared.
 *
 * Supported: Annex-B byte stream, SPS/PPS/slice header, I and P slices,
 * CAVLC, I_PCM, Intra4x4/Intra16x16/chroma intra, P_L0 16x16/16x8/8x16/8x8
 * (all sub types), P_Skip, multiple reference frames, short and long term
 * references, ref pic list modification, MMCO, multiple slices,
 * constrained_intra_pred, deblocking (idc 0/1/2 and offsets), cropping.
 * Not supported: CABAC, B slices, FMO/ASO, interlace, 8x8 transform,
 * scaling matrices, weighted prediction, frame num gaps.
 *
 * Usage: h264_model in.264 out.yuv [-t trace.txt] [-nocrop]
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>

#define MAX_DPB 18

static void die(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    fprintf(stderr, "error: ");
    vfprintf(stderr, fmt, ap);
    fprintf(stderr, "\n");
    va_end(ap);
    exit(1);
}

static inline int clip3(int lo, int hi, int v) { return v < lo ? lo : (v > hi ? hi : v); }
static inline int clip1(int v) { return clip3(0, 255, v); }
static inline int iabs(int v) { return v < 0 ? -v : v; }
static inline int imin(int a, int b) { return a < b ? a : b; }
static inline int imax(int a, int b) { return a > b ? a : b; }
static inline int median3(int a, int b, int c) { return imax(imin(a, b), imin(imax(a, b), c)); }

/* ------------------------------------------------------------------ */
/* Tables (ITU-T H.264 clause 9 / 8)                                  */
/* ------------------------------------------------------------------ */

/* coeff_token, index = total_coeff*4 + trailing_ones */
static const uint8_t coeff_token_len[4][4 * 17] = {
{ 1, 0, 0, 0,  6, 2, 0, 0,  8, 6, 3, 0,  9, 8, 7, 5, 10, 9, 8, 6,
 11,10, 9, 7, 13,11,10, 8, 13,13,11, 9, 13,13,13,10, 14,14,13,11,
 14,14,14,13, 15,15,14,14, 15,15,15,14, 16,15,15,15, 16,16,16,15,
 16,16,16,16, 16,16,16,16 },
{ 2, 0, 0, 0,  6, 2, 0, 0,  6, 5, 3, 0,  7, 6, 6, 4,  8, 6, 6, 4,
  8, 7, 7, 5,  9, 8, 8, 6, 11, 9, 9, 6, 11,11,11, 7, 12,11,11, 9,
 12,12,12,11, 12,12,12,11, 13,13,13,12, 13,13,13,13, 13,14,13,13,
 14,14,14,13, 14,14,14,14 },
{ 4, 0, 0, 0,  6, 4, 0, 0,  6, 5, 4, 0,  6, 5, 5, 4,  7, 5, 5, 4,
  7, 5, 5, 4,  7, 6, 6, 4,  7, 6, 6, 4,  8, 7, 7, 5,  8, 8, 7, 6,
  9, 8, 8, 7,  9, 9, 8, 8,  9, 9, 9, 8, 10, 9, 9, 9, 10,10,10,10,
 10,10,10,10, 10,10,10,10 },
{ 6, 0, 0, 0,  6, 6, 0, 0,  6, 6, 6, 0,  6, 6, 6, 6,  6, 6, 6, 6,
  6, 6, 6, 6,  6, 6, 6, 6,  6, 6, 6, 6,  6, 6, 6, 6,  6, 6, 6, 6,
  6, 6, 6, 6,  6, 6, 6, 6,  6, 6, 6, 6,  6, 6, 6, 6,  6, 6, 6, 6,
  6, 6, 6, 6,  6, 6, 6, 6 }
};
static const uint8_t coeff_token_bits[4][4 * 17] = {
{ 1, 0, 0, 0,  5, 1, 0, 0,  7, 4, 1, 0,  7, 6, 5, 3,  7, 6, 5, 3,
  7, 6, 5, 4, 15, 6, 5, 4, 11,14, 5, 4,  8,10,13, 4, 15,14, 9, 4,
 11,10,13,12, 15,14, 9,12, 11,10,13, 8, 15, 1, 9,12, 11,14,13, 8,
  7,10, 9,12,  4, 6, 5, 8 },
{ 3, 0, 0, 0, 11, 2, 0, 0,  7, 7, 3, 0,  7,10, 9, 5,  7, 6, 5, 4,
  4, 6, 5, 6,  7, 6, 5, 8, 15, 6, 5, 4, 11,14,13, 4, 15,10, 9, 4,
 11,14,13,12,  8,10, 9, 8, 15,14,13,12, 11,10, 9,12,  7,11, 6, 8,
  9, 8,10, 1,  7, 6, 5, 4 },
{15, 0, 0, 0, 15,14, 0, 0, 11,15,13, 0,  8,12,14,12, 15,10,11,11,
 11, 8, 9,10,  9,14,13, 9,  8,10, 9, 8, 15,14,13,13, 11,14,10,12,
 15,10,13,12, 11,14, 9,12,  8,10,13, 8, 13, 7, 9,12,  9,12,11,10,
  5, 8, 7, 6,  1, 4, 3, 2 },
{ 3, 0, 0, 0,  0, 1, 0, 0,  4, 5, 6, 0,  8, 9,10,11, 12,13,14,15,
 16,17,18,19, 20,21,22,23, 24,25,26,27, 28,29,30,31, 32,33,34,35,
 36,37,38,39, 40,41,42,43, 44,45,46,47, 48,49,50,51, 52,53,54,55,
 56,57,58,59, 60,61,62,63 }
};
static const uint8_t chroma_dc_coeff_token_len[4 * 5] = {
 2, 0, 0, 0,  6, 1, 0, 0,  6, 6, 3, 0,  6, 7, 7, 6,  6, 8, 8, 7 };
static const uint8_t chroma_dc_coeff_token_bits[4 * 5] = {
 1, 0, 0, 0,  7, 1, 0, 0,  4, 6, 1, 0,  3, 3, 2, 5,  2, 3, 2, 0 };

/* total_zeros, [total_coeff-1][total_zeros] */
static const uint8_t total_zeros_len[15][16] = {
 {1,3,3,4,4,5,5,6,6,7,7,8,8,9,9,9}, {3,3,3,3,3,4,4,4,4,5,5,6,6,6,6},
 {4,3,3,3,4,4,3,3,4,5,5,6,5,6}, {5,3,4,4,3,3,3,4,3,4,5,5,5},
 {4,4,4,3,3,3,3,3,4,5,4,5}, {6,5,3,3,3,3,3,3,4,3,6}, {6,5,3,3,3,2,3,4,3,6},
 {6,4,5,3,2,2,3,3,6}, {6,6,4,2,2,3,2,5}, {5,5,3,2,2,2,4}, {4,4,3,3,1,3},
 {4,4,2,1,3}, {3,3,1,2}, {2,2,1}, {1,1} };
static const uint8_t total_zeros_bits[15][16] = {
 {1,3,2,3,2,3,2,3,2,3,2,3,2,3,2,1}, {7,6,5,4,3,5,4,3,2,3,2,3,2,1,0},
 {5,7,6,5,4,3,4,3,2,3,2,1,1,0}, {3,7,5,4,6,5,4,3,3,2,2,1,0},
 {5,4,3,7,6,5,4,3,2,1,1,0}, {1,1,7,6,5,4,3,2,1,1,0}, {1,1,5,4,3,3,2,1,1,0},
 {1,1,1,3,3,2,2,1,0}, {1,0,1,3,2,1,1,1}, {1,0,1,3,2,1,1}, {0,1,1,2,1,3},
 {0,1,1,1,1}, {0,1,1,1}, {0,1,1}, {0,1} };
static const uint8_t chroma_dc_total_zeros_len[3][4] = { {1,2,3,3}, {1,2,2,0}, {1,1,0,0} };
static const uint8_t chroma_dc_total_zeros_bits[3][4] = { {1,1,1,0}, {1,1,0,0}, {1,0,0,0} };

/* run_before, [min(zeros_left,7)-1][run_before] */
static const uint8_t run_len[7][16] = {
 {1,1}, {1,2,2}, {2,2,2,2}, {2,2,2,3,3}, {2,2,3,3,3,3}, {2,3,3,3,3,3,3},
 {3,3,3,3,3,3,3,4,5,6,7,8,9,10,11} };
static const uint8_t run_bits[7][16] = {
 {1,0}, {1,1,0}, {3,2,1,0}, {3,2,1,1,0}, {3,2,3,2,1,0}, {3,0,1,3,2,5,4},
 {7,6,5,4,3,2,1,1,1,1,1,1,1,1,1} };

static const uint8_t golomb_to_intra4x4_cbp[48] = {
 47,31,15, 0,23,27,29,30, 7,11,13,14,39,43,45,46,16, 3, 5,10,12,19,21,26,
 28,35,37,42,44, 1, 2, 4, 8,17,18,20,24, 6, 9,22,25,32,33,34,36,40,38,41 };
static const uint8_t golomb_to_inter_cbp[48] = {
  0,16, 1, 2, 4, 8,32, 3, 5,10,12,15,47, 7,11,13,14, 6, 9,31,35,37,42,44,
 33,34,36,40,39,43,45,46,17,18,20,24,19,21,26,28,23,27,29,30,22,25,38,41 };

/* zig-zag scan, scan position -> raster index (x + 4*y) */
static const uint8_t zigzag4x4[16] = { 0, 1, 4, 8, 5, 2, 3, 6, 9, 12, 13, 10, 7, 11, 14, 15 };

/* luma4x4BlkIdx -> block x,y (in 4x4 units) */
static const uint8_t blk_x[16] = { 0,1,0,1, 2,3,2,3, 0,1,0,1, 2,3,2,3 };
static const uint8_t blk_y[16] = { 0,0,1,1, 0,0,1,1, 2,2,3,3, 2,2,3,3 };

static const uint8_t dequant_v[6][3] = { /* both even, mixed, both odd */
 {10,13,16}, {11,14,18}, {13,16,20}, {14,18,23}, {16,20,25}, {18,23,29} };

static const uint8_t chroma_qp_tab[52] = {
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,
 29,30,31,32,32,33,34,34,35,35,36,36,37,37,37,38,38,38,39,39,39,39 };

static const uint8_t alpha_tab[52] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 4, 5, 6, 7, 8, 9,10,12,13,
 15,17,20,22,25,28,32,36,40,45,50,56,63,71,80,90,101,113,127,144,162,182,203,226,255,255 };
static const uint8_t beta_tab[52] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4,
  6, 6, 7, 7, 8, 8, 9, 9,10,10,11,11,12,12,13,13,14,14,15,15,16,16,17,17,18,18 };
static const uint8_t tc0_tab[52][3] = {
 {0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},
 {0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,1},{0,0,1},{0,0,1},
 {0,0,1},{0,1,1},{0,1,1},{1,1,1},{1,1,1},{1,1,1},{1,1,1},{1,1,2},{1,1,2},{1,1,2},
 {1,1,2},{1,2,3},{1,2,3},{2,2,3},{2,2,4},{2,3,4},{2,3,4},{3,3,5},{3,4,6},{3,4,6},
 {4,5,7},{4,5,8},{4,6,9},{5,7,10},{6,8,11},{6,8,13},{7,10,14},{8,11,16},{9,12,18},
 {10,13,20},{11,15,23},{13,17,25} };

/* ------------------------------------------------------------------ */
/* Bit reader over an RBSP (emulation prevention already removed)     */
/* ------------------------------------------------------------------ */
typedef struct {
    const uint8_t *buf;
    int size;      /* bytes */
    int pos;       /* bit position */
    int end_bit;   /* bit position of rbsp_stop_one_bit */
} BitReader;

static void br_init(BitReader *br, const uint8_t *buf, int size)
{
    br->buf = buf;
    br->size = size;
    br->pos = 0;
    br->end_bit = size * 8;
    /* locate rbsp_stop_one_bit: last 1 bit of the last non-zero byte */
    int i = size - 1;
    while (i >= 0 && buf[i] == 0) i--;
    if (i >= 0) {
        int b = buf[i], tz = 0;
        while (!(b & 1)) { b >>= 1; tz++; }
        br->end_bit = i * 8 + 7 - tz;
    }
}

static uint32_t br_peek(BitReader *br, int n)
{
    uint32_t v = 0;
    for (int i = 0; i < n; i++) {
        int p = br->pos + i;
        int bit = (p >> 3) < br->size ? (br->buf[p >> 3] >> (7 - (p & 7))) & 1 : 0;
        v = (v << 1) | bit;
    }
    return v;
}
static uint32_t br_u(BitReader *br, int n) { uint32_t v = br_peek(br, n); br->pos += n; return v; }
static int br_u1(BitReader *br) { return br_u(br, 1); }
static uint32_t br_ue(BitReader *br)
{
    int lz = 0;
    while (br_u1(br) == 0) { lz++; if (lz > 31) die("bad exp-golomb"); }
    if (lz == 0) return 0;
    return ((1u << lz) - 1) + br_u(br, lz);
}
static int br_se(BitReader *br)
{
    uint32_t k = br_ue(br);
    return (k & 1) ? (int)((k + 1) >> 1) : -(int)(k >> 1);
}
static int br_more_rbsp_data(BitReader *br) { return br->pos < br->end_bit; }
static int br_te(BitReader *br, int range)
{
    if (range == 1) return !br_u1(br);
    return br_ue(br);
}

/* ------------------------------------------------------------------ */
/* Parameter sets                                                     */
/* ------------------------------------------------------------------ */
typedef struct {
    int valid;
    int profile_idc, level_idc;
    int log2_max_frame_num;
    int poc_type, log2_max_poc_lsb, delta_pic_order_always_zero;
    int num_ref_frames, gaps_allowed;
    int mb_width, mb_height;
    int frame_mbs_only;
    int crop_left, crop_right, crop_top, crop_bottom;
} SPS;

typedef struct {
    int valid;
    int sps_id;
    int entropy_coding_mode;
    int bottom_field_pic_order_in_frame_present;
    int num_slice_groups;
    int num_ref_idx_l0_default;
    int weighted_pred;
    int pic_init_qp;
    int chroma_qp_index_offset;
    int deblocking_filter_control_present;
    int constrained_intra_pred;
    int redundant_pic_cnt_present;
    int transform_8x8_mode;
} PPS;

static SPS sps_tab[32];
static PPS pps_tab[256];

static void parse_sps(BitReader *br)
{
    SPS s;
    memset(&s, 0, sizeof(s));
    s.profile_idc = br_u(br, 8);
    br_u(br, 8); /* constraint flags + reserved */
    s.level_idc = br_u(br, 8);
    int id = br_ue(br);
    if (id > 31) die("sps id");
    if (s.profile_idc == 100 || s.profile_idc == 110 || s.profile_idc == 122 || s.profile_idc == 244 ||
        s.profile_idc == 44 || s.profile_idc == 83 || s.profile_idc == 86 || s.profile_idc == 118 ||
        s.profile_idc == 128 || s.profile_idc == 138 || s.profile_idc == 139 || s.profile_idc == 134 ||
        s.profile_idc == 135) {
        int cf = br_ue(br);
        if (cf != 1) die("only 4:2:0 supported");
        if (br_ue(br) != 0 || br_ue(br) != 0) die("only 8 bit supported");
        if (br_u1(br)) die("lossless (transform bypass) not supported");
        if (br_u1(br)) die("scaling matrices not supported");
    }
    s.log2_max_frame_num = br_ue(br) + 4;
    s.poc_type = br_ue(br);
    if (s.poc_type == 0) {
        s.log2_max_poc_lsb = br_ue(br) + 4;
    } else if (s.poc_type == 1) {
        s.delta_pic_order_always_zero = br_u1(br);
        br_se(br); br_se(br);
        int n = br_ue(br);
        for (int i = 0; i < n; i++) br_se(br);
    }
    s.num_ref_frames = br_ue(br);
    s.gaps_allowed = br_u1(br);
    s.mb_width = br_ue(br) + 1;
    s.mb_height = br_ue(br) + 1;
    s.frame_mbs_only = br_u1(br);
    if (!s.frame_mbs_only) die("interlace not supported");
    br_u1(br); /* direct_8x8_inference */
    if (br_u1(br)) {
        s.crop_left = br_ue(br);
        s.crop_right = br_ue(br);
        s.crop_top = br_ue(br);
        s.crop_bottom = br_ue(br);
    }
    /* VUI ignored */
    s.valid = 1;
    sps_tab[id] = s;
}

static void parse_pps(BitReader *br)
{
    PPS p;
    memset(&p, 0, sizeof(p));
    int id = br_ue(br);
    if (id > 255) die("pps id");
    p.sps_id = br_ue(br);
    p.entropy_coding_mode = br_u1(br);
    p.bottom_field_pic_order_in_frame_present = br_u1(br);
    p.num_slice_groups = br_ue(br) + 1;
    if (p.num_slice_groups != 1) die("FMO not supported");
    p.num_ref_idx_l0_default = br_ue(br) + 1;
    br_ue(br); /* l1 */
    p.weighted_pred = br_u1(br);
    br_u(br, 2);
    p.pic_init_qp = 26 + br_se(br);
    br_se(br); /* pic_init_qs */
    p.chroma_qp_index_offset = br_se(br);
    p.deblocking_filter_control_present = br_u1(br);
    p.constrained_intra_pred = br_u1(br);
    p.redundant_pic_cnt_present = br_u1(br);
    if (br_more_rbsp_data(br)) {
        p.transform_8x8_mode = br_u1(br);
        if (br_u1(br)) die("pps scaling matrices not supported");
        int second = br_se(br);
        if (second != p.chroma_qp_index_offset) die("second chroma qp offset not supported");
    }
    p.valid = 1;
    pps_tab[id] = p;
}

/* ------------------------------------------------------------------ */
/* Frames and DPB                                                     */
/* ------------------------------------------------------------------ */
enum { REF_NONE = 0, REF_SHORT = 1, REF_LONG = 2 };

typedef struct {
    uint8_t *plane[3];
    int frame_num, frame_num_wrap, pic_num;
    int ref;            /* REF_NONE / REF_SHORT / REF_LONG */
    int long_term_idx;
    int in_use;         /* holds a reference or the current picture */
} Frame;

static Frame dpb[MAX_DPB];
static int max_long_term_idx = -1; /* -1: no long-term frame indices */

/* ------------------------------------------------------------------ */
/* Per-MB info                                                        */
/* ------------------------------------------------------------------ */
enum {
    MB_I4x4, MB_I16x16, MB_IPCM, MB_P16x16, MB_P16x8, MB_P8x16, MB_P8x8, MB_PSKIP
};
#define IS_INTRA(t) ((t) <= MB_IPCM)

typedef struct {
    int type;
    int part, sub[4];
    int tc_ldc;
    int slice_num;
    int qp;              /* QP_Y (qp pred for next mb) */
    int qp_db;           /* qp used by deblocking (0 for I_PCM) */
    int qpc_db;          /* chroma qp used by deblocking */
    int i16_mode, chroma_mode, cbp;
    int i4mode[16];      /* raster 4x4 order */
    int tc_luma[16];     /* total_coeff per luma 4x4, raster order */
    int tc_chroma[2][4]; /* total_coeff per chroma 4x4, raster order */
    int ref_idx[16];     /* per 4x4 raster */
    int ref_pic[16];     /* dpb index per 4x4, -1 none */
    int mv[16][2];
    /* slice deblocking params */
    int disable_db, alpha_off, beta_off;
    /* residual */
    int coef_luma[16][16];  /* raster block, raster coeff position (levels) */
    int coef_ldc[16];       /* Intra16x16 DC levels, raster position */
    int coef_cdc[2][4];
    int coef_cac[2][4][16];
    uint8_t pcm[384];
} MbInfo;

/* ------------------------------------------------------------------ */
/* Decoder state                                                      */
/* ------------------------------------------------------------------ */
typedef struct {
    int first_mb, slice_type, pps_id, frame_num, idr, nal_ref_idc;
    int num_ref_idx_active;
    int qp;
    int disable_db, alpha_off, beta_off;
    int long_term_reference_flag;
    int adaptive_marking;
    int mmco_op[66], mmco_a[66], mmco_count;
} SliceHdr;

static SPS *sps;
static PPS *pps;
static int mb_w, mb_h, width, height;
static MbInfo *mbs;
static int cur_frame = -1;    /* dpb index of picture being decoded */
static int mbs_decoded;
static int slice_num;
static int ref_list[33];
static int ref_list_len;
static int prev_ref_frame_num;
static FILE *trace;
static FILE *ptrace;   /* parser token trace, same format as the RTL testbench */
static FILE *rtrace;   /* unfiltered reconstruction per macroblock */
static FILE *fout;
static int nocrop;
static int frames_out;
static int cur_pic_has_mmco5;
static SliceHdr pic_sh; /* header of first slice of current picture (marking) */

/* ------------------------------------------------------------------ */
/* Neighbour helpers                                                  */
/* ------------------------------------------------------------------ */
/* Availability of macroblock (mbx,mby) relative to current macroblock in slice */
static int mb_avail(int mbx, int mby, int cur_slice)
{
    if (mbx < 0 || mby < 0 || mbx >= mb_w || mby >= mb_h) return 0;
    MbInfo *m = &mbs[mby * mb_w + mbx];
    return m->slice_num == cur_slice;
}

static int bs_compute(MbInfo *p, int pb, MbInfo *q, int qb, int mb_edge);
static int intra_avail(int mbx, int mby, int cur_slice);

/* ------------------------------------------------------------------ */
/* CAVLC                                                              */
/* ------------------------------------------------------------------ */
static int read_coeff_token(BitReader *br, int nC, int *trailing_ones)
{
    const uint8_t *len, *bits;
    int n;
    if (nC == -1) { len = chroma_dc_coeff_token_len; bits = chroma_dc_coeff_token_bits; n = 5; }
    else {
        int t = nC < 2 ? 0 : nC < 4 ? 1 : nC < 8 ? 2 : 3;
        len = coeff_token_len[t]; bits = coeff_token_bits[t]; n = 17;
    }
    for (int i = 0; i < n * 4; i++) {
        int l = len[i];
        if (!l) continue;
        if (br_peek(br, l) == bits[i]) {
            br->pos += l;
            *trailing_ones = i & 3;
            return i >> 2;
        }
    }
    die("bad coeff_token at bit %d", br->pos);
    return 0;
}

static int read_vlc(BitReader *br, const uint8_t *len, const uint8_t *bits, int n)
{
    for (int i = 0; i < n; i++) {
        int l = len[i];
        if (!l) continue;
        if (br_peek(br, l) == bits[i]) { br->pos += l; return i; }
    }
    die("bad vlc at bit %d", br->pos);
    return 0;
}

/* residual_block_cavlc: coeffLevel[startIdx..endIdx] in scan order.
 * out[] receives levels at scan positions 0..maxNum-1 (relative). */
static int residual_block(BitReader *br, int nC, int maxNum, int *out)
{
    int level[16], run[16];
    int t1, tc;
    for (int i = 0; i < maxNum; i++) out[i] = 0;
    tc = read_coeff_token(br, nC, &t1);
    if (tc == 0) return 0;
    if (tc > maxNum) die("total_coeff > max");
    int suffix_length = (tc > 10 && t1 < 3) ? 1 : 0;
    for (int i = 0; i < tc; i++) {
        if (i < t1) {
            level[i] = br_u1(br) ? -1 : 1;
        } else {
            int prefix = 0;
            while (br_u1(br) == 0) { prefix++; if (prefix > 32) die("level prefix"); }
            int level_code = (imin(15, prefix) << suffix_length);
            int suffix_size;
            if (prefix == 14 && suffix_length == 0) suffix_size = 4;
            else if (prefix >= 15) suffix_size = prefix - 3;
            else suffix_size = suffix_length;
            if (suffix_size > 0) level_code += br_u(br, suffix_size);
            if (prefix >= 15 && suffix_length == 0) level_code += 15;
            if (prefix >= 16) level_code += (1 << (prefix - 3)) - 4096;
            if (i == t1 && t1 < 3) level_code += 2;
            int lv = (level_code % 2 == 0) ? (level_code + 2) >> 1 : (-level_code - 1) >> 1;
            level[i] = lv;
            if (suffix_length == 0) suffix_length = 1;
            if (iabs(lv) > (3 << (suffix_length - 1)) && suffix_length < 6) suffix_length++;
        }
    }
    int zeros_left;
    if (tc < maxNum) {
        if (maxNum == 4)
            zeros_left = read_vlc(br, chroma_dc_total_zeros_len[tc - 1], chroma_dc_total_zeros_bits[tc - 1], 4);
        else
            zeros_left = read_vlc(br, total_zeros_len[tc - 1], total_zeros_bits[tc - 1], 16);
    } else zeros_left = 0;
    for (int i = 0; i < tc - 1; i++) {
        if (zeros_left > 0) {
            int t = imin(zeros_left, 7) - 1;
            run[i] = read_vlc(br, run_len[t], run_bits[t], 16);
        } else run[i] = 0;
        zeros_left -= run[i];
        if (zeros_left < 0) die("negative zeros_left");
    }
    run[tc - 1] = zeros_left;
    int coeff_num = -1;
    for (int i = tc - 1; i >= 0; i--) {
        coeff_num += run[i] + 1;
        out[coeff_num] = level[i];
    }
    return tc;
}

/* total_coeff of neighbour luma block for nC; bx,by may be -1 */
static int luma_nc_get(int mbx, int mby, int bx, int by, int cur_slice, int *avail)
{
    if (bx < 0) { mbx--; bx += 4; }
    if (by < 0) { mby--; by += 4; }
    if (!mb_avail(mbx, mby, cur_slice)) { *avail = 0; return 0; }
    *avail = 1;
    MbInfo *m = &mbs[mby * mb_w + mbx];
    if (m->type == MB_PSKIP) return 0;
    if (m->type == MB_IPCM) return 16;
    return m->tc_luma[by * 4 + bx];
}
static int chroma_nc_get(int mbx, int mby, int c, int bx, int by, int cur_slice, int *avail)
{
    if (bx < 0) { mbx--; bx += 2; }
    if (by < 0) { mby--; by += 2; }
    if (!mb_avail(mbx, mby, cur_slice)) { *avail = 0; return 0; }
    *avail = 1;
    MbInfo *m = &mbs[mby * mb_w + mbx];
    if (m->type == MB_PSKIP) return 0;
    if (m->type == MB_IPCM) return 16;
    return m->tc_chroma[c][by * 2 + bx];
}
static int calc_nc(int na, int aa, int nb, int ab)
{
    if (aa && ab) return (na + nb + 1) >> 1;
    if (aa) return na;
    if (ab) return nb;
    return 0;
}

/* ------------------------------------------------------------------ */
/* Motion vector prediction                                           */
/* ------------------------------------------------------------------ */
#define NOT_AVAIL (-2)

typedef struct { int ref; int mv[2]; } Nb;

/* neighbour at luma 4x4 coordinate (x4,y4) relative to current MB top-left
 * (range -1..4). decoded_mask marks 4x4 blocks of current MB already done. */
static Nb get_nb(int mbx, int mby, int x4, int y4, int cur_slice, MbInfo *cur, unsigned decoded_mask)
{
    Nb n;
    n.ref = NOT_AVAIL; n.mv[0] = n.mv[1] = 0;
    if (x4 >= 0 && x4 < 4 && y4 >= 0 && y4 < 4) {
        int b = y4 * 4 + x4;
        if (!(decoded_mask & (1u << b))) return n;
        n.ref = cur->ref_idx[b];
        n.mv[0] = cur->mv[b][0]; n.mv[1] = cur->mv[b][1];
        return n;
    }
    if (x4 > 3 && y4 >= 0) return n; /* right MB not yet decoded */
    int nmx = mbx + (x4 < 0 ? -1 : (x4 > 3 ? 1 : 0));
    int nmy = mby + (y4 < 0 ? -1 : 0);
    if (!mb_avail(nmx, nmy, cur_slice)) return n;
    MbInfo *m = &mbs[nmy * mb_w + nmx];
    int bx = (x4 + 4) & 3, by = (y4 + 4) & 3;
    if (IS_INTRA(m->type)) { n.ref = -1; return n; }
    int b = by * 4 + bx;
    n.ref = m->ref_idx[b];
    n.mv[0] = m->mv[b][0]; n.mv[1] = m->mv[b][1];
    return n;
}

/* x4,y4,w4,h4 in 4x4 units. shape: 0 generic, 1 16x8, 2 8x16 */
static void mv_pred(int mbx, int mby, int cur_slice, MbInfo *cur, unsigned mask,
                    int x4, int y4, int w4, int shape, int part, int ref, int *mvp)
{
    Nb A = get_nb(mbx, mby, x4 - 1, y4, cur_slice, cur, mask);
    Nb B = get_nb(mbx, mby, x4, y4 - 1, cur_slice, cur, mask);
    Nb C = get_nb(mbx, mby, x4 + w4, y4 - 1, cur_slice, cur, mask);
    if (C.ref == NOT_AVAIL) C = get_nb(mbx, mby, x4 - 1, y4 - 1, cur_slice, cur, mask);

    if (shape == 1) {
        if (part == 0 && B.ref == ref) { mvp[0] = B.mv[0]; mvp[1] = B.mv[1]; return; }
        if (part == 1 && A.ref == ref) { mvp[0] = A.mv[0]; mvp[1] = A.mv[1]; return; }
    } else if (shape == 2) {
        if (part == 0 && A.ref == ref) { mvp[0] = A.mv[0]; mvp[1] = A.mv[1]; return; }
        if (part == 1 && C.ref == ref) { mvp[0] = C.mv[0]; mvp[1] = C.mv[1]; return; }
    }
    if (B.ref == NOT_AVAIL && C.ref == NOT_AVAIL && A.ref != NOT_AVAIL) {
        B = A; C = A;
    }
    int match = (A.ref == ref) + (B.ref == ref) + (C.ref == ref);
    if (match == 1) {
        Nb *s = (A.ref == ref) ? &A : (B.ref == ref) ? &B : &C;
        mvp[0] = s->mv[0]; mvp[1] = s->mv[1];
    } else {
        mvp[0] = median3(A.mv[0], B.mv[0], C.mv[0]);
        mvp[1] = median3(A.mv[1], B.mv[1], C.mv[1]);
    }
}

static void set_part(MbInfo *m, int x4, int y4, int w4, int h4, int ref, const int *mv, unsigned *mask)
{
    for (int y = y4; y < y4 + h4; y++)
        for (int x = x4; x < x4 + w4; x++) {
            int b = y * 4 + x;
            m->ref_idx[b] = ref;
            m->ref_pic[b] = ref >= 0 ? ref_list[ref] : -1;
            m->mv[b][0] = mv[0]; m->mv[b][1] = mv[1];
            *mask |= 1u << b;
        }
}

/* ------------------------------------------------------------------ */
/* Macroblock layer parsing                                           */
/* ------------------------------------------------------------------ */
static void trace_mb(int addr, MbInfo *m, int bitpos)
{
    if (!trace) return;
    fprintf(trace, "MB %d type %d qp %d cbp %d bit %d\n", addr, m->type, m->qp, m->cbp, bitpos);
}

static void decode_mb_pskip(int mbx, int mby, SliceHdr *sh, int qp)
{
    MbInfo *m = &mbs[mby * mb_w + mbx];
    int cur_slice = slice_num;
    memset(m->tc_luma, 0, sizeof(m->tc_luma));
    memset(m->tc_chroma, 0, sizeof(m->tc_chroma));
    memset(m->coef_luma, 0, sizeof(m->coef_luma));
    memset(m->coef_cdc, 0, sizeof(m->coef_cdc));
    memset(m->coef_cac, 0, sizeof(m->coef_cac));
    m->type = MB_PSKIP;
    m->part = 0; m->tc_ldc = 0;
    for (int i = 0; i < 4; i++) m->sub[i] = 0;
    m->qp = qp;
    m->cbp = 0;
    int mv[2] = { 0, 0 };
    int availA = mb_avail(mbx - 1, mby, cur_slice);
    int availB = mb_avail(mbx, mby - 1, cur_slice);
    unsigned mask = 0;
    if (availA && availB) {
        Nb A = get_nb(mbx, mby, -1, 0, cur_slice, m, 0);
        Nb B = get_nb(mbx, mby, 0, -1, cur_slice, m, 0);
        if (!((A.ref == 0 && A.mv[0] == 0 && A.mv[1] == 0) || (B.ref == 0 && B.mv[0] == 0 && B.mv[1] == 0)))
            mv_pred(mbx, mby, cur_slice, m, 0, 0, 0, 4, 0, 0, 0, mv);
    }
    set_part(m, 0, 0, 4, 4, 0, mv, &mask);
    (void)sh;
}

static int pred_i4mode(int mbx, int mby, MbInfo *m, int bx, int by, int cur_slice)
{
    int modes[2];
    int dcflag = 0;
    for (int k = 0; k < 2; k++) {
        int x = bx - (k == 0), y = by - (k == 1);
        int nmx = mbx, nmy = mby;
        if (x < 0) { nmx--; x += 4; }
        if (y < 0) { nmy--; y += 4; }
        if (nmx == mbx && nmy == mby) { modes[k] = m->i4mode[y * 4 + x]; continue; }
        if (!mb_avail(nmx, nmy, cur_slice)) { dcflag = 1; modes[k] = 2; continue; }
        MbInfo *n = &mbs[nmy * mb_w + nmx];
        if (!IS_INTRA(n->type) && pps->constrained_intra_pred) { dcflag = 1; modes[k] = 2; continue; }
        modes[k] = (n->type == MB_I4x4) ? n->i4mode[y * 4 + x] : 2;
    }
    if (dcflag) return 2;
    return imin(modes[0], modes[1]);
}

static void parse_residual(BitReader *br, int mbx, int mby, MbInfo *m)
{
    int cur_slice = slice_num;
    int tmp[16];
    memset(m->coef_luma, 0, sizeof(m->coef_luma));
    memset(m->coef_ldc, 0, sizeof(m->coef_ldc));
    memset(m->coef_cdc, 0, sizeof(m->coef_cdc));
    memset(m->coef_cac, 0, sizeof(m->coef_cac));
    memset(m->tc_luma, 0, sizeof(m->tc_luma));
    memset(m->tc_chroma, 0, sizeof(m->tc_chroma));

    if (m->type == MB_I16x16) {
        int aa, ab;
        int na = luma_nc_get(mbx, mby, -1, 0, cur_slice, &aa);
        int nb = luma_nc_get(mbx, mby, 0, -1, cur_slice, &ab);
        m->tc_ldc = residual_block(br, calc_nc(na, aa, nb, ab), 16, tmp);
        for (int i = 0; i < 16; i++) m->coef_ldc[zigzag4x4[i]] = tmp[i];
    }
    for (int b8 = 0; b8 < 4; b8++) {
        for (int b4 = 0; b4 < 4; b4++) {
            int blk = b8 * 4 + b4;
            int bx = blk_x[blk], by = blk_y[blk];
            int r = by * 4 + bx;
            if (!(m->cbp & (1 << b8))) { m->tc_luma[r] = 0; continue; }
            int aa, ab, na, nb;
            if (bx > 0) { aa = 1; na = m->tc_luma[r - 1]; }
            else na = luma_nc_get(mbx, mby, -1, by, cur_slice, &aa);
            if (by > 0) { ab = 1; nb = m->tc_luma[r - 4]; }
            else nb = luma_nc_get(mbx, mby, bx, -1, cur_slice, &ab);
            int nc = calc_nc(na, aa, nb, ab);
            if (m->type == MB_I16x16) {
                int tc = residual_block(br, nc, 15, tmp);
                m->tc_luma[r] = tc;
                for (int i = 0; i < 15; i++) m->coef_luma[r][zigzag4x4[i + 1]] = tmp[i];
            } else {
                int tc = residual_block(br, nc, 16, tmp);
                m->tc_luma[r] = tc;
                for (int i = 0; i < 16; i++) m->coef_luma[r][zigzag4x4[i]] = tmp[i];
            }
        }
    }
    int cbpc = m->cbp >> 4;
    if (cbpc & 3) {
        for (int c = 0; c < 2; c++) {
            residual_block(br, -1, 4, tmp);
            for (int i = 0; i < 4; i++) m->coef_cdc[c][i] = tmp[i];
        }
    }
    if (cbpc & 2) {
        for (int c = 0; c < 2; c++) {
            for (int b = 0; b < 4; b++) {
                int bx = b & 1, by = b >> 1;
                int aa, ab, na, nb;
                if (bx > 0) { aa = 1; na = m->tc_chroma[c][b - 1]; }
                else na = chroma_nc_get(mbx, mby, c, -1, by, cur_slice, &aa);
                if (by > 0) { ab = 1; nb = m->tc_chroma[c][b - 2]; }
                else nb = chroma_nc_get(mbx, mby, c, bx, -1, cur_slice, &ab);
                int tc = residual_block(br, calc_nc(na, aa, nb, ab), 15, tmp);
                m->tc_chroma[c][b] = tc;
                for (int i = 0; i < 15; i++) m->coef_cac[c][b][zigzag4x4[i + 1]] = tmp[i];
            }
        }
    }
}

static void decode_mb(BitReader *br, int mbx, int mby, SliceHdr *sh, int *qp)
{
    MbInfo *m = &mbs[mby * mb_w + mbx];
    int cur_slice = slice_num;
    int bitpos = br->pos;
    int mb_type = br_ue(br);
    int is_p = (sh->slice_type == 0);
    unsigned mask = 0;

    m->cbp = 0;
    m->part = 0; m->tc_ldc = 0;
    for (int i = 0; i < 4; i++) m->sub[i] = 0;
    for (int i = 0; i < 16; i++) { m->ref_idx[i] = -1; m->ref_pic[i] = -1; m->mv[i][0] = m->mv[i][1] = 0; m->i4mode[i] = 2; }

    if (is_p && mb_type < 5) {
        int nparts = 0;
        int refs[4] = { 0, 0, 0, 0 };
        int range = sh->num_ref_idx_active - 1;
        if (mb_type == 0) { m->type = MB_P16x16; nparts = 1; }
        else if (mb_type == 1) { m->type = MB_P16x8; nparts = 2; }
        else if (mb_type == 2) { m->type = MB_P8x16; nparts = 2; }
        else { m->type = MB_P8x8; nparts = 4; }
        m->part = mb_type < 3 ? mb_type : 3;
        if (m->type != MB_P8x8) {
            for (int p = 0; p < nparts; p++) refs[p] = range > 0 ? br_te(br, range) : 0;
            for (int p = 0; p < nparts; p++) {
                int mvd[2], mvp[2], mv[2];
                mvd[0] = br_se(br); mvd[1] = br_se(br);
                int x4, y4, w4, h4, shape;
                if (m->type == MB_P16x16) { x4 = 0; y4 = 0; w4 = 4; h4 = 4; shape = 0; }
                else if (m->type == MB_P16x8) { x4 = 0; y4 = p * 2; w4 = 4; h4 = 2; shape = 1; }
                else { x4 = p * 2; y4 = 0; w4 = 2; h4 = 4; shape = 2; }
                mv_pred(mbx, mby, cur_slice, m, mask, x4, y4, w4, shape, p, refs[p], mvp);
                mv[0] = mvp[0] + mvd[0]; mv[1] = mvp[1] + mvd[1];
                set_part(m, x4, y4, w4, h4, refs[p], mv, &mask);
            }
        } else {
            int sub[4];
            for (int i = 0; i < 4; i++) { sub[i] = br_ue(br); if (sub[i] > 3) die("sub_mb_type"); m->sub[i] = sub[i]; }
            for (int i = 0; i < 4; i++) refs[i] = (range > 0 && mb_type == 3) ? br_te(br, range) : 0;
            for (int i = 0; i < 4; i++) {
                int bx8 = (i & 1) * 2, by8 = (i >> 1) * 2;
                int n = sub[i] == 0 ? 1 : sub[i] == 3 ? 4 : 2;
                for (int j = 0; j < n; j++) {
                    int mvd[2], mvp[2], mv[2];
                    int x4, y4, w4, h4;
                    mvd[0] = br_se(br); mvd[1] = br_se(br);
                    switch (sub[i]) {
                    case 0: x4 = bx8; y4 = by8; w4 = 2; h4 = 2; break;
                    case 1: x4 = bx8; y4 = by8 + j; w4 = 2; h4 = 1; break;
                    case 2: x4 = bx8 + j; y4 = by8; w4 = 1; h4 = 2; break;
                    default: x4 = bx8 + (j & 1); y4 = by8 + (j >> 1); w4 = 1; h4 = 1; break;
                    }
                    mv_pred(mbx, mby, cur_slice, m, mask, x4, y4, w4, 0, 0, refs[i], mvp);
                    mv[0] = mvp[0] + mvd[0]; mv[1] = mvp[1] + mvd[1];
                    set_part(m, x4, y4, w4, h4, refs[i], mv, &mask);
                }
            }
        }
        int code = br_ue(br);
        if (code > 47) die("cbp");
        m->cbp = golomb_to_inter_cbp[code];
    } else {
        int it = is_p ? mb_type - 5 : mb_type;
        if (it > 25) die("mb_type %d", mb_type);
        if (it == 0) {
            m->type = MB_I4x4;
            for (int blk = 0; blk < 16; blk++) {
                int bx = blk_x[blk], by = blk_y[blk];
                int pred = pred_i4mode(mbx, mby, m, bx, by, cur_slice);
                int mode;
                if (br_u1(br)) mode = pred;
                else {
                    int rem = br_u(br, 3);
                    mode = rem < pred ? rem : rem + 1;
                }
                m->i4mode[by * 4 + bx] = mode;
            }
            m->chroma_mode = br_ue(br);
            int code = br_ue(br);
            if (code > 47) die("cbp");
            m->cbp = golomb_to_intra4x4_cbp[code];
        } else if (it == 25) {
            m->type = MB_IPCM;
            while (br->pos & 7) br_u1(br);
            for (int i = 0; i < 384; i++) m->pcm[i] = br_u(br, 8);
            for (int i = 0; i < 16; i++) m->tc_luma[i] = 16;
            for (int c = 0; c < 2; c++) for (int i = 0; i < 4; i++) m->tc_chroma[c][i] = 16;
            m->qp = *qp;
            m->cbp = 0x2f;
            trace_mb(mby * mb_w + mbx, m, bitpos);
            return;
        } else {
            m->type = MB_I16x16;
            m->i16_mode = (it - 1) & 3;
            int cc = ((it - 1) >> 2) % 3;
            int cl = (it >= 13) ? 15 : 0;
            m->cbp = cl | (cc << 4);
            m->chroma_mode = br_ue(br);
        }
        if (m->chroma_mode > 3) die("chroma mode");
    }
    if ((m->cbp & 0x3f) || m->type == MB_I16x16) {
        int dqp = br_se(br);
        if (dqp < -26 || dqp > 25) die("mb_qp_delta");
        *qp = (*qp + dqp + 52) % 52;
    }
    m->qp = *qp;
    parse_residual(br, mbx, mby, m);
    trace_mb(mby * mb_w + mbx, m, bitpos);
}

/* ------------------------------------------------------------------ */
/* Reconstruction                                                     */
/* ------------------------------------------------------------------ */
static uint8_t *P(int c) { return dpb[cur_frame].plane[c]; }
static int pw(int c) { return c ? width / 2 : width; }
static int ph(int c) { return c ? height / 2 : height; }

static void idct4x4(int *d, int *r)
{
    int t[16];
    for (int i = 0; i < 4; i++) {
        int *row = &d[i * 4];
        int e0 = row[0] + row[2], e1 = row[0] - row[2];
        int e2 = (row[1] >> 1) - row[3], e3 = row[1] + (row[3] >> 1);
        t[i * 4 + 0] = e0 + e3; t[i * 4 + 1] = e1 + e2;
        t[i * 4 + 2] = e1 - e2; t[i * 4 + 3] = e0 - e3;
    }
    for (int j = 0; j < 4; j++) {
        int g0 = t[0 * 4 + j] + t[2 * 4 + j], g1 = t[0 * 4 + j] - t[2 * 4 + j];
        int g2 = (t[1 * 4 + j] >> 1) - t[3 * 4 + j], g3 = t[1 * 4 + j] + (t[3 * 4 + j] >> 1);
        r[0 * 4 + j] = (g0 + g3 + 32) >> 6; r[1 * 4 + j] = (g1 + g2 + 32) >> 6;
        r[2 * 4 + j] = (g1 - g2 + 32) >> 6; r[3 * 4 + j] = (g0 - g3 + 32) >> 6;
    }
}

static int dq_v(int qp, int pos)
{
    int x = pos & 3, y = pos >> 2;
    return dequant_v[qp % 6][(x & 1) + (y & 1)];
}

/* builds residual of 4x4 block: coef levels (raster), optional dc already scaled */
static void residual4x4(const int *coef, int qp, int has_dc, int dc, int *res)
{
    int d[16];
    for (int i = 0; i < 16; i++) d[i] = (coef[i] * dq_v(qp, i)) << (qp / 6);
    if (has_dc) d[0] = dc;
    idct4x4(d, res);
}

static void luma_dc_dequant(const int *c, int qp, int *dc)
{
    int t[16], f[16];
    for (int i = 0; i < 4; i++) {
        const int *r = &c[i * 4];
        t[i * 4 + 0] = r[0] + r[1] + r[2] + r[3];
        t[i * 4 + 1] = r[0] + r[1] - r[2] - r[3];
        t[i * 4 + 2] = r[0] - r[1] - r[2] + r[3];
        t[i * 4 + 3] = r[0] - r[1] + r[2] - r[3];
    }
    for (int j = 0; j < 4; j++) {
        int a = t[0 * 4 + j], b = t[1 * 4 + j], cc = t[2 * 4 + j], d = t[3 * 4 + j];
        f[0 * 4 + j] = a + b + cc + d;
        f[1 * 4 + j] = a + b - cc - d;
        f[2 * 4 + j] = a - b - cc + d;
        f[3 * 4 + j] = a - b + cc - d;
    }
    int ls = 16 * dequant_v[qp % 6][0];
    for (int i = 0; i < 16; i++) {
        if (qp >= 36) dc[i] = (f[i] * ls) << (qp / 6 - 6);
        else dc[i] = (f[i] * ls + (1 << (5 - qp / 6))) >> (6 - qp / 6);
    }
}

static void chroma_dc_dequant(const int *c, int qp, int *dc)
{
    int f[4];
    f[0] = c[0] + c[1] + c[2] + c[3];
    f[1] = c[0] - c[1] + c[2] - c[3];
    f[2] = c[0] + c[1] - c[2] - c[3];
    f[3] = c[0] - c[1] - c[2] + c[3];
    int ls = 16 * dequant_v[qp % 6][0];
    for (int i = 0; i < 4; i++) dc[i] = ((f[i] * ls) << (qp / 6)) >> 5;
}

/* neighbour sample availability for intra prediction (constrained intra aware) */
static int intra_avail(int mbx, int mby, int cur_slice)
{
    if (!mb_avail(mbx, mby, cur_slice)) return 0;
    MbInfo *m = &mbs[mby * mb_w + mbx];
    if (pps->constrained_intra_pred && !IS_INTRA(m->type)) return 0;
    return 1;
}

static void intra4x4_pred(int mode, int *top /* 8 + [-1] at index -1 */, int *left, int availT, int availL,
                          int availTL, int *pred)
{
    (void)availTL;
#define T(x) top[x]
#define L(y) left[y]
    for (int y = 0; y < 4; y++)
        for (int x = 0; x < 4; x++) {
            int v = 0;
            switch (mode) {
            case 0: v = T(x); break;
            case 1: v = L(y); break;
            case 2:
                if (availT && availL) v = (T(0) + T(1) + T(2) + T(3) + L(0) + L(1) + L(2) + L(3) + 4) >> 3;
                else if (availL) v = (L(0) + L(1) + L(2) + L(3) + 2) >> 2;
                else if (availT) v = (T(0) + T(1) + T(2) + T(3) + 2) >> 2;
                else v = 128;
                break;
            case 3:
                if (x == 3 && y == 3) v = (T(6) + 3 * T(7) + 2) >> 2;
                else v = (T(x + y) + 2 * T(x + y + 1) + T(x + y + 2) + 2) >> 2;
                break;
            case 4:
                if (x > y) v = (T(x - y - 2) + 2 * T(x - y - 1) + T(x - y) + 2) >> 2;
                else if (x < y) v = (L(y - x - 2) + 2 * L(y - x - 1) + L(y - x) + 2) >> 2;
                else v = (T(0) + 2 * T(-1) + L(0) + 2) >> 2;
                break;
            case 5: {
                int z = 2 * x - y;
                if (z >= 0 && !(z & 1)) v = (T(x - (y >> 1) - 1) + T(x - (y >> 1)) + 1) >> 1;
                else if (z >= 0) v = (T(x - (y >> 1) - 2) + 2 * T(x - (y >> 1) - 1) + T(x - (y >> 1)) + 2) >> 2;
                else if (z == -1) v = (L(0) + 2 * T(-1) + T(0) + 2) >> 2;
                else v = (L(y - 1) + 2 * L(y - 2) + L(y - 3) + 2) >> 2;
                break;
            }
            case 6: {
                int z = 2 * y - x;
                if (z >= 0 && !(z & 1)) v = (L(y - (x >> 1) - 1) + L(y - (x >> 1)) + 1) >> 1;
                else if (z >= 0) v = (L(y - (x >> 1) - 2) + 2 * L(y - (x >> 1) - 1) + L(y - (x >> 1)) + 2) >> 2;
                else if (z == -1) v = (L(0) + 2 * T(-1) + T(0) + 2) >> 2;
                else v = (T(x - 1) + 2 * T(x - 2) + T(x - 3) + 2) >> 2;
                break;
            }
            case 7:
                if (!(y & 1)) v = (T(x + (y >> 1)) + T(x + (y >> 1) + 1) + 1) >> 1;
                else v = (T(x + (y >> 1)) + 2 * T(x + (y >> 1) + 1) + T(x + (y >> 1) + 2) + 2) >> 2;
                break;
            case 8: {
                int z = x + 2 * y;
                if (z < 5 && !(z & 1)) v = (L(y + (x >> 1)) + L(y + (x >> 1) + 1) + 1) >> 1;
                else if (z < 5) v = (L(y + (x >> 1)) + 2 * L(y + (x >> 1) + 1) + L(y + (x >> 1) + 2) + 2) >> 2;
                else if (z == 5) v = (L(2) + 3 * L(3) + 2) >> 2;
                else v = L(3);
                break;
            }
            default: die("i4 mode");
            }
            pred[y * 4 + x] = v;
        }
#undef T
#undef L
}

/* left[-1] is the top-left sample: callers pass pointers into arrays with offset 1 */
static void recon_intra4x4(int mbx, int mby, MbInfo *m)
{
    uint8_t *Y = P(0);
    int W = width;
    int cur_slice = m->slice_num;
    int availA = intra_avail(mbx - 1, mby, cur_slice);
    int availB = intra_avail(mbx, mby - 1, cur_slice);
    int availC = intra_avail(mbx + 1, mby - 1, cur_slice);
    int availD = intra_avail(mbx - 1, mby - 1, cur_slice);
    for (int blk = 0; blk < 16; blk++) {
        int bx = blk_x[blk], by = blk_y[blk];
        int px = mbx * 16 + bx * 4, py = mby * 16 + by * 4;
        int topbuf[9], leftbuf5[5];
        int *top = topbuf + 1, *leftbuf = leftbuf5 + 1;
        int availT = by > 0 ? 1 : availB;
        int availL = bx > 0 ? 1 : availA;
        int availTL = (bx > 0 && by > 0) ? 1 : (bx == 0 && by == 0) ? availD : (bx == 0 ? availA : availB);
        int availTR;
        if (by == 0) availTR = (bx < 3) ? availB : availC;
        else if (bx == 3) availTR = 0;
        else {
            /* block (bx+1, by-1) decoded before current? */
            int nb = -1;
            for (int k = 0; k < 16; k++) if (blk_x[k] == bx + 1 && blk_y[k] == by - 1) nb = k;
            availTR = nb < blk;
        }
        for (int i = 0; i < 4; i++) {
            top[i] = availT ? Y[(py - 1) * W + px + i] : 0;
            leftbuf[i] = availL ? Y[(py + i) * W + px - 1] : 0;
        }
        for (int i = 4; i < 8; i++) top[i] = availTR ? Y[(py - 1) * W + px + i] : top[3];
        top[-1] = availTL ? Y[(py - 1) * W + px - 1] : 0;
        leftbuf[-1] = top[-1];
        int pred[16], res[16];
        intra4x4_pred(m->i4mode[by * 4 + bx], top, leftbuf, availT, availL, availTL, pred);
        int r = by * 4 + bx;
        residual4x4(m->coef_luma[r], m->qp, 0, 0, res);
        for (int y = 0; y < 4; y++)
            for (int x = 0; x < 4; x++)
                Y[(py + y) * W + px + x] = clip1(pred[y * 4 + x] + res[y * 4 + x]);
    }
}

static void recon_intra16x16(int mbx, int mby, MbInfo *m)
{
    uint8_t *Y = P(0);
    int W = width;
    int cur_slice = m->slice_num;
    int availA = intra_avail(mbx - 1, mby, cur_slice);
    int availB = intra_avail(mbx, mby - 1, cur_slice);
    int availD = intra_avail(mbx - 1, mby - 1, cur_slice);
    int px = mbx * 16, py = mby * 16;
    int top[17], left[17]; /* index 0 = -1 */
    top[0] = left[0] = availD ? Y[(py - 1) * W + px - 1] : 0;
    for (int i = 0; i < 16; i++) {
        top[i + 1] = availB ? Y[(py - 1) * W + px + i] : 0;
        left[i + 1] = availA ? Y[(py + i) * W + px - 1] : 0;
    }
    int pred[256];
    int mode = m->i16_mode;
    if (mode == 0) { for (int y = 0; y < 16; y++) for (int x = 0; x < 16; x++) pred[y * 16 + x] = top[x + 1]; }
    else if (mode == 1) { for (int y = 0; y < 16; y++) for (int x = 0; x < 16; x++) pred[y * 16 + x] = left[y + 1]; }
    else if (mode == 2) {
        int s = 0, v;
        if (availA && availB) { for (int i = 0; i < 16; i++) s += top[i + 1] + left[i + 1]; v = (s + 16) >> 5; }
        else if (availA) { for (int i = 0; i < 16; i++) s += left[i + 1]; v = (s + 8) >> 4; }
        else if (availB) { for (int i = 0; i < 16; i++) s += top[i + 1]; v = (s + 8) >> 4; }
        else v = 128;
        for (int i = 0; i < 256; i++) pred[i] = v;
    } else {
        int H = 0, V = 0;
        for (int i = 0; i < 8; i++) {
            H += (i + 1) * (top[8 + i + 1] - top[6 - i + 1]);
            V += (i + 1) * (left[8 + i + 1] - left[6 - i + 1]);
        }
        int a = 16 * (left[16] + top[16]);
        int b = (5 * H + 32) >> 6, c = (5 * V + 32) >> 6;
        for (int y = 0; y < 16; y++)
            for (int x = 0; x < 16; x++)
                pred[y * 16 + x] = clip1((a + b * (x - 7) + c * (y - 7) + 16) >> 5);
    }
    int dc[16];
    luma_dc_dequant(m->coef_ldc, m->qp, dc);
    for (int by = 0; by < 4; by++)
        for (int bx = 0; bx < 4; bx++) {
            int res[16];
            int r = by * 4 + bx;
            residual4x4(m->coef_luma[r], m->qp, 1, dc[r], res);
            for (int y = 0; y < 4; y++)
                for (int x = 0; x < 4; x++) {
                    int yy = by * 4 + y, xx = bx * 4 + x;
                    Y[(py + yy) * W + px + xx] = clip1(pred[yy * 16 + xx] + res[y * 4 + x]);
                }
        }
}

static void chroma_intra_pred(int mbx, int mby, MbInfo *m, int c, int *pred)
{
    uint8_t *C = P(1 + c);
    int W = width / 2;
    int cur_slice = m->slice_num;
    int availA = intra_avail(mbx - 1, mby, cur_slice);
    int availB = intra_avail(mbx, mby - 1, cur_slice);
    int availD = intra_avail(mbx - 1, mby - 1, cur_slice);
    int px = mbx * 8, py = mby * 8;
    int top[9], left[9];
    top[0] = left[0] = availD ? C[(py - 1) * W + px - 1] : 0;
    for (int i = 0; i < 8; i++) {
        top[i + 1] = availB ? C[(py - 1) * W + px + i] : 0;
        left[i + 1] = availA ? C[(py + i) * W + px - 1] : 0;
    }
    int mode = m->chroma_mode;
    if (mode == 0) {
        for (int b = 0; b < 4; b++) {
            int xo = (b & 1) * 4, yo = (b >> 1) * 4;
            int st = 0, sl = 0, v;
            for (int i = 0; i < 4; i++) { st += top[xo + i + 1]; sl += left[yo + i + 1]; }
            if ((xo == 0 && yo == 0) || (xo == 4 && yo == 4)) {
                if (availA && availB) v = (st + sl + 4) >> 3;
                else if (availA) v = (sl + 2) >> 2;
                else if (availB) v = (st + 2) >> 2;
                else v = 128;
            } else if (xo == 4 && yo == 0) {
                if (availB) v = (st + 2) >> 2;
                else if (availA) v = (sl + 2) >> 2;
                else v = 128;
            } else {
                if (availA) v = (sl + 2) >> 2;
                else if (availB) v = (st + 2) >> 2;
                else v = 128;
            }
            for (int y = 0; y < 4; y++) for (int x = 0; x < 4; x++) pred[(yo + y) * 8 + xo + x] = v;
        }
    } else if (mode == 1) {
        for (int y = 0; y < 8; y++) for (int x = 0; x < 8; x++) pred[y * 8 + x] = left[y + 1];
    } else if (mode == 2) {
        for (int y = 0; y < 8; y++) for (int x = 0; x < 8; x++) pred[y * 8 + x] = top[x + 1];
    } else {
        int H = 0, V = 0;
        for (int i = 0; i < 4; i++) {
            H += (i + 1) * (top[4 + i + 1] - top[2 - i + 1]);
            V += (i + 1) * (left[4 + i + 1] - left[2 - i + 1]);
        }
        int a = 16 * (left[8] + top[8]);
        int b = (34 * H + 32) >> 6, cc = (34 * V + 32) >> 6;
        for (int y = 0; y < 8; y++)
            for (int x = 0; x < 8; x++)
                pred[y * 8 + x] = clip1((a + b * (x - 3) + cc * (y - 3) + 16) >> 5);
    }
}

static int ref_luma(Frame *f, int x, int y)
{
    x = clip3(0, width - 1, x); y = clip3(0, height - 1, y);
    return f->plane[0][y * width + x];
}
static int tap6(int a, int b, int c, int d, int e, int f) { return a - 5 * b + 20 * c + 20 * d - 5 * e + f; }

static int luma_interp(Frame *f, int xi, int yi, int xf, int yf)
{
#define R(dx, dy) ref_luma(f, xi + (dx), yi + (dy))
    int G = R(0, 0);
    if (xf == 0 && yf == 0) return G;
    int b1 = tap6(R(-2, 0), R(-1, 0), R(0, 0), R(1, 0), R(2, 0), R(3, 0));
    int h1 = tap6(R(0, -2), R(0, -1), R(0, 0), R(0, 1), R(0, 2), R(0, 3));
    int s1 = tap6(R(-2, 1), R(-1, 1), R(0, 1), R(1, 1), R(2, 1), R(3, 1));
    int m1 = tap6(R(1, -2), R(1, -1), R(1, 0), R(1, 1), R(1, 2), R(1, 3));
    int b = clip1((b1 + 16) >> 5), h = clip1((h1 + 16) >> 5);
    int s = clip1((s1 + 16) >> 5), mm = clip1((m1 + 16) >> 5);
    int col[6];
    for (int k = 0; k < 6; k++)
        col[k] = tap6(R(-2, k - 2), R(-1, k - 2), R(0, k - 2), R(1, k - 2), R(2, k - 2), R(3, k - 2));
    int j1 = tap6(col[0], col[1], col[2], col[3], col[4], col[5]);
    int j = clip1((j1 + 512) >> 10);
    int Hs = R(1, 0), Ms = R(0, 1);
#undef R
    switch (yf * 4 + xf) {
    case 1: return (G + b + 1) >> 1;      /* a */
    case 2: return b;
    case 3: return (Hs + b + 1) >> 1;     /* c */
    case 4: return (G + h + 1) >> 1;      /* d */
    case 5: return (b + h + 1) >> 1;      /* e */
    case 6: return (b + j + 1) >> 1;      /* f */
    case 7: return (b + mm + 1) >> 1;     /* g */
    case 8: return h;
    case 9: return (h + j + 1) >> 1;      /* i */
    case 10: return j;
    case 11: return (j + mm + 1) >> 1;    /* k */
    case 12: return (Ms + h + 1) >> 1;    /* n */
    case 13: return (h + s + 1) >> 1;     /* p */
    case 14: return (j + s + 1) >> 1;     /* q */
    case 15: return (mm + s + 1) >> 1;    /* r */
    }
    return G;
}

static void recon_inter(int mbx, int mby, MbInfo *m)
{
    for (int b = 0; b < 16; b++) {
        int bx = b & 3, by = b >> 2;
        Frame *f = &dpb[m->ref_pic[b]];
        int mvx = m->mv[b][0], mvy = m->mv[b][1];
        int px = mbx * 16 + bx * 4, py = mby * 16 + by * 4;
        int res[16];
        int r = b;
        residual4x4(m->coef_luma[r], m->qp, 0, 0, res);
        for (int y = 0; y < 4; y++)
            for (int x = 0; x < 4; x++) {
                int xa = (px + x) * 4 + mvx, ya = (py + y) * 4 + mvy;
                int v = luma_interp(f, xa >> 2, ya >> 2, xa & 3, ya & 3);
                P(0)[(py + y) * width + px + x] = clip1(v + res[y * 4 + x]);
            }
    }
}

static void inter_chroma_pred(MbInfo *m, int mbx, int mby, int c, int *pred)
{
    int W = width / 2, H = height / 2;
    for (int y = 0; y < 8; y++)
        for (int x = 0; x < 8; x++) {
            int b = (y >> 1) * 4 + (x >> 1);
            Frame *f = &dpb[m->ref_pic[b]];
            uint8_t *C = f->plane[1 + c];
            int xa = (mbx * 8 + x) * 8 + m->mv[b][0], ya = (mby * 8 + y) * 8 + m->mv[b][1];
            int xi = xa >> 3, yi = ya >> 3, xf = xa & 7, yf = ya & 7;
            int x0 = clip3(0, W - 1, xi), x1 = clip3(0, W - 1, xi + 1);
            int y0 = clip3(0, H - 1, yi), y1 = clip3(0, H - 1, yi + 1);
            int A = C[y0 * W + x0], B = C[y0 * W + x1], Cc = C[y1 * W + x0], D = C[y1 * W + x1];
            pred[y * 8 + x] = ((8 - xf) * (8 - yf) * A + xf * (8 - yf) * B + (8 - xf) * yf * Cc + xf * yf * D + 32) >> 6;
        }
}

static void recon_chroma(int mbx, int mby, MbInfo *m)
{
    int qpc = chroma_qp_tab[clip3(0, 51, m->qp + pps->chroma_qp_index_offset)];
    for (int c = 0; c < 2; c++) {
        int pred[64];
        if (IS_INTRA(m->type)) chroma_intra_pred(mbx, mby, m, c, pred);
        else inter_chroma_pred(m, mbx, mby, c, pred);
        int dc[4];
        chroma_dc_dequant(m->coef_cdc[c], qpc, dc);
        uint8_t *C = P(1 + c);
        int W = width / 2;
        for (int b = 0; b < 4; b++) {
            int bx = b & 1, by = b >> 1;
            int res[16];
            residual4x4(m->coef_cac[c][b], qpc, 1, dc[b], res);
            for (int y = 0; y < 4; y++)
                for (int x = 0; x < 4; x++) {
                    int yy = by * 4 + y, xx = bx * 4 + x;
                    C[(mby * 8 + yy) * W + mbx * 8 + xx] = clip1(pred[yy * 8 + xx] + res[y * 4 + x]);
                }
        }
    }
}

static void rtrace_mb(int mbx, int mby)
{
    if (!rtrace) return;
    fprintf(rtrace, "R %d", mby * mb_w + mbx);
    for (int y = 0; y < 16; y++) for (int x = 0; x < 16; x++)
        fprintf(rtrace, " %d", P(0)[(mby * 16 + y) * width + mbx * 16 + x]);
    for (int c = 0; c < 2; c++)
        for (int y = 0; y < 8; y++) for (int x = 0; x < 8; x++)
            fprintf(rtrace, " %d", P(1 + c)[(mby * 8 + y) * (width / 2) + mbx * 8 + x]);
    fprintf(rtrace, "\n");
}

static void recon_mb_(int mbx, int mby);
static void recon_mb(int mbx, int mby)
{
    recon_mb_(mbx, mby);
    rtrace_mb(mbx, mby);
}

static void recon_mb_(int mbx, int mby)
{
    MbInfo *m = &mbs[mby * mb_w + mbx];
    if (m->type == MB_IPCM) {
        for (int y = 0; y < 16; y++) for (int x = 0; x < 16; x++)
            P(0)[(mby * 16 + y) * width + mbx * 16 + x] = m->pcm[y * 16 + x];
        for (int c = 0; c < 2; c++)
            for (int y = 0; y < 8; y++) for (int x = 0; x < 8; x++)
                P(1 + c)[(mby * 8 + y) * (width / 2) + mbx * 8 + x] = m->pcm[256 + c * 64 + y * 8 + x];
        return;
    }
    if (m->type == MB_I4x4) recon_intra4x4(mbx, mby, m);
    else if (m->type == MB_I16x16) recon_intra16x16(mbx, mby, m);
    else recon_inter(mbx, mby, m);
    recon_chroma(mbx, mby, m);
}

/* ------------------------------------------------------------------ */
/* Deblocking                                                         */
/* ------------------------------------------------------------------ */
static int bs_compute(MbInfo *p, int pb, MbInfo *q, int qb, int mb_edge)
{
    if (IS_INTRA(p->type) || IS_INTRA(q->type)) return mb_edge ? 4 : 3;
    if (p->tc_luma[pb] || q->tc_luma[qb]) return 2;
    if (p->ref_pic[pb] != q->ref_pic[qb]) return 1;
    if (iabs(p->mv[pb][0] - q->mv[qb][0]) >= 4 || iabs(p->mv[pb][1] - q->mv[qb][1]) >= 4) return 1;
    return 0;
}

static void filter_line(uint8_t *s, int step, int bs, int alpha, int beta, int tc0, int chroma)
{
    int p0 = s[-step], p1 = s[-2 * step], q0 = s[0], q1 = s[step];
    if (!(iabs(p0 - q0) < alpha && iabs(p1 - p0) < beta && iabs(q1 - q0) < beta)) return;
    if (chroma) {
        if (bs < 4) {
            int tc = tc0 + 1;
            int delta = clip3(-tc, tc, (((q0 - p0) << 2) + (p1 - q1) + 4) >> 3);
            s[-step] = clip1(p0 + delta);
            s[0] = clip1(q0 - delta);
        } else {
            s[-step] = (2 * p1 + p0 + q1 + 2) >> 2;
            s[0] = (2 * q1 + q0 + p1 + 2) >> 2;
        }
        return;
    }
    int p2 = s[-3 * step], q2 = s[2 * step];
    int ap = iabs(p2 - p0), aq = iabs(q2 - q0);
    if (bs < 4) {
        int tc = tc0 + (ap < beta) + (aq < beta);
        int delta = clip3(-tc, tc, (((q0 - p0) << 2) + (p1 - q1) + 4) >> 3);
        s[-step] = clip1(p0 + delta);
        s[0] = clip1(q0 - delta);
        if (ap < beta) s[-2 * step] = p1 + clip3(-tc0, tc0, (p2 + ((p0 + q0 + 1) >> 1) - (p1 << 1)) >> 1);
        if (aq < beta) s[step] = q1 + clip3(-tc0, tc0, (q2 + ((p0 + q0 + 1) >> 1) - (q1 << 1)) >> 1);
    } else {
        int p3 = s[-4 * step], q3 = s[3 * step];
        int strong = iabs(p0 - q0) < ((alpha >> 2) + 2);
        if (ap < beta && strong) {
            s[-step] = (p2 + 2 * p1 + 2 * p0 + 2 * q0 + q1 + 4) >> 3;
            s[-2 * step] = (p2 + p1 + p0 + q0 + 2) >> 2;
            s[-3 * step] = (2 * p3 + 3 * p2 + p1 + p0 + q0 + 4) >> 3;
        } else s[-step] = (2 * p1 + p0 + q1 + 2) >> 2;
        if (aq < beta && strong) {
            s[0] = (p1 + 2 * p0 + 2 * q0 + 2 * q1 + q2 + 4) >> 3;
            s[step] = (p0 + q0 + q1 + q2 + 2) >> 2;
            s[2 * step] = (2 * q3 + 3 * q2 + q1 + q0 + p0 + 4) >> 3;
        } else s[0] = (2 * q1 + q0 + p1 + 2) >> 2;
    }
}

static void deblock_mb(int mbx, int mby)
{
    MbInfo *q = &mbs[mby * mb_w + mbx];
    if (q->disable_db == 1) return;
    for (int dir = 0; dir < 2; dir++) { /* 0: vertical edges, 1: horizontal edges */
        for (int e = 0; e < 4; e++) {
            MbInfo *p = q;
            int mb_edge = (e == 0);
            if (mb_edge) {
                int nmx = mbx - (dir == 0), nmy = mby - (dir == 1);
                if (nmx < 0 || nmy < 0) continue;
                p = &mbs[nmy * mb_w + nmx];
                if (q->disable_db == 2 && p->slice_num != q->slice_num) continue;
            }
            int bs[4];
            for (int k = 0; k < 4; k++) {
                int qb = dir == 0 ? k * 4 + e : e * 4 + k;
                int pb;
                if (dir == 0) pb = mb_edge ? k * 4 + 3 : qb - 1;
                else pb = mb_edge ? 12 + k : qb - 4;
                bs[k] = bs_compute(p, pb, q, qb, mb_edge);
            }
            /* luma */
            int qpav = (p->qp_db + q->qp_db + 1) >> 1;
            int ia = clip3(0, 51, qpav + q->alpha_off), ib = clip3(0, 51, qpav + q->beta_off);
            int alpha = alpha_tab[ia], beta = beta_tab[ib];
            uint8_t *Y = P(0);
            for (int k = 0; k < 16; k++) {
                int b = bs[k >> 2];
                if (!b) continue;
                int x, y, step;
                if (dir == 0) { x = mbx * 16 + e * 4; y = mby * 16 + k; step = 1; }
                else { x = mbx * 16 + k; y = mby * 16 + e * 4; step = width; }
                filter_line(&Y[y * width + x], step, b, alpha, beta, b < 4 ? tc0_tab[ia][b - 1] : 0, 0);
            }
            /* chroma: luma edges 0 and 2 */
            if (e & 1) continue;
            int qpavc = (p->qpc_db + q->qpc_db + 1) >> 1;
            int iac = clip3(0, 51, qpavc + q->alpha_off), ibc = clip3(0, 51, qpavc + q->beta_off);
            int alphac = alpha_tab[iac], betac = beta_tab[ibc];
            for (int c = 0; c < 2; c++) {
                uint8_t *C = P(1 + c);
                int W = width / 2;
                for (int k = 0; k < 8; k++) {
                    int b = bs[k >> 1];
                    if (!b) continue;
                    int x, y, step;
                    if (dir == 0) { x = mbx * 8 + e * 2; y = mby * 8 + k; step = 1; }
                    else { x = mbx * 8 + k; y = mby * 8 + e * 2; step = W; }
                    filter_line(&C[y * W + x], step, b, alphac, betac, b < 4 ? tc0_tab[iac][b - 1] : 0, 1);
                }
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/* Reference lists and marking                                        */
/* ------------------------------------------------------------------ */
static void build_ref_list(SliceHdr *sh, BitReader *br)
{
    int max_frame_num = 1 << sps->log2_max_frame_num;
    int cur_frame_num = sh->frame_num;
    int list[40], n = 0;
    for (int i = 0; i < MAX_DPB; i++) {
        Frame *f = &dpb[i];
        if (i == cur_frame) continue;
        if (f->ref == REF_SHORT) {
            f->frame_num_wrap = f->frame_num > cur_frame_num ? f->frame_num - max_frame_num : f->frame_num;
            f->pic_num = f->frame_num_wrap;
        }
    }
    /* short-term, descending PicNum */
    int used[MAX_DPB] = { 0 };
    for (;;) {
        int best = -1;
        for (int i = 0; i < MAX_DPB; i++)
            if (i != cur_frame && !used[i] && dpb[i].ref == REF_SHORT &&
                (best < 0 || dpb[i].pic_num > dpb[best].pic_num)) best = i;
        if (best < 0) break;
        used[best] = 1;
        list[n++] = best;
    }
    for (;;) {
        int best = -1;
        for (int i = 0; i < MAX_DPB; i++)
            if (i != cur_frame && !used[i] && dpb[i].ref == REF_LONG &&
                (best < 0 || dpb[i].long_term_idx < dpb[best].long_term_idx)) best = i;
        if (best < 0) break;
        used[best] = 1;
        list[n++] = best;
    }
    int len = sh->num_ref_idx_active;
    for (int i = n; i < 34; i++) list[i] = -1;
    /* ref_pic_list_modification */
    if (br_u1(br)) {
        int pic_num_pred = cur_frame_num;
        int ref_idx = 0;
        for (;;) {
            int idc = br_ue(br);
            if (idc == 3) break;
            if (idc > 3) die("modification idc");
            int target = -1;
            if (idc < 2) {
                int abs_diff = br_ue(br) + 1;
                int no_wrap;
                if (idc == 0) { no_wrap = pic_num_pred - abs_diff; if (no_wrap < 0) no_wrap += max_frame_num; }
                else { no_wrap = pic_num_pred + abs_diff; if (no_wrap >= max_frame_num) no_wrap -= max_frame_num; }
                pic_num_pred = no_wrap;
                int pic_num = no_wrap > cur_frame_num ? no_wrap - max_frame_num : no_wrap;
                for (int i = 0; i < MAX_DPB; i++)
                    if (i != cur_frame && dpb[i].ref == REF_SHORT && dpb[i].pic_num == pic_num) target = i;
                if (target < 0) die("modification: short pic not found");
                for (int c = len; c > ref_idx; c--) list[c] = list[c - 1];
                list[ref_idx++] = target;
                int nidx = ref_idx;
                for (int c = ref_idx; c <= len; c++)
                    if (!(list[c] >= 0 && dpb[list[c]].ref == REF_SHORT && dpb[list[c]].pic_num == pic_num))
                        list[nidx++] = list[c];
            } else {
                int lt = br_ue(br);
                for (int i = 0; i < MAX_DPB; i++)
                    if (i != cur_frame && dpb[i].ref == REF_LONG && dpb[i].long_term_idx == lt) target = i;
                if (target < 0) die("modification: long pic not found");
                for (int c = len; c > ref_idx; c--) list[c] = list[c - 1];
                list[ref_idx++] = target;
                int nidx = ref_idx;
                for (int c = ref_idx; c <= len; c++)
                    if (!(list[c] >= 0 && dpb[list[c]].ref == REF_LONG && dpb[list[c]].long_term_idx == lt))
                        list[nidx++] = list[c];
            }
        }
    }
    for (int i = 0; i < len; i++) ref_list[i] = list[i];
    ref_list_len = len;
    if (trace) {
        fprintf(trace, "REFLIST");
        for (int i = 0; i < len; i++) fprintf(trace, " %d", ref_list[i]);
        fprintf(trace, "\n");
    }
}

static void parse_dec_ref_pic_marking(BitReader *br, SliceHdr *sh)
{
    sh->mmco_count = 0;
    sh->adaptive_marking = 0;
    sh->long_term_reference_flag = 0;
    if (sh->idr) {
        br_u1(br); /* no_output_of_prior_pics */
        sh->long_term_reference_flag = br_u1(br);
    } else {
        sh->adaptive_marking = br_u1(br);
        if (sh->adaptive_marking) {
            for (;;) {
                int op = br_ue(br);
                if (op == 0) break;
                if (op > 6) die("mmco");
                int a = 0, b = 0;
                if (op == 1 || op == 3) a = br_ue(br) + 1;          /* difference_of_pic_nums */
                if (op == 2) a = br_ue(br);                         /* long_term_pic_num */
                if (op == 3 || op == 6) b = br_ue(br);              /* long_term_frame_idx */
                if (op == 4) a = br_ue(br);                         /* max_long_term_frame_idx_plus1 */
                if (op == 3) { sh->mmco_op[sh->mmco_count] = op; sh->mmco_a[sh->mmco_count] = a | (b << 16); }
                else if (op == 6) { sh->mmco_op[sh->mmco_count] = op; sh->mmco_a[sh->mmco_count] = b; }
                else { sh->mmco_op[sh->mmco_count] = op; sh->mmco_a[sh->mmco_count] = a; }
                sh->mmco_count++;
                if (sh->mmco_count >= 66) die("too many mmco");
            }
        }
    }
}

static void mark_picture(SliceHdr *sh)
{
    Frame *cur = &dpb[cur_frame];
    int max_frame_num = 1 << sps->log2_max_frame_num;
    cur_pic_has_mmco5 = 0;
    if (sh->nal_ref_idc == 0) { cur->ref = REF_NONE; return; }
    if (sh->idr) {
        for (int i = 0; i < MAX_DPB; i++) if (i != cur_frame) dpb[i].ref = REF_NONE;
        if (sh->long_term_reference_flag) { cur->ref = REF_LONG; cur->long_term_idx = 0; max_long_term_idx = 0; }
        else { cur->ref = REF_SHORT; max_long_term_idx = -1; }
        return;
    }
    int cur_long = 0;
    if (sh->adaptive_marking) {
        for (int k = 0; k < sh->mmco_count; k++) {
            int op = sh->mmco_op[k], a = sh->mmco_a[k];
            for (int i = 0; i < MAX_DPB; i++)
                if (i != cur_frame && dpb[i].ref == REF_SHORT) {
                    int fnw = dpb[i].frame_num > sh->frame_num ? dpb[i].frame_num - max_frame_num : dpb[i].frame_num;
                    dpb[i].pic_num = fnw;
                }
            if (op == 1) {
                int pn = sh->frame_num - a;
                for (int i = 0; i < MAX_DPB; i++)
                    if (i != cur_frame && dpb[i].ref == REF_SHORT && dpb[i].pic_num == pn) dpb[i].ref = REF_NONE;
            } else if (op == 2) {
                for (int i = 0; i < MAX_DPB; i++)
                    if (i != cur_frame && dpb[i].ref == REF_LONG && dpb[i].long_term_idx == a) dpb[i].ref = REF_NONE;
            } else if (op == 3) {
                int pn = sh->frame_num - (a & 0xffff), lti = a >> 16;
                for (int i = 0; i < MAX_DPB; i++)
                    if (i != cur_frame && dpb[i].ref == REF_LONG && dpb[i].long_term_idx == lti) dpb[i].ref = REF_NONE;
                for (int i = 0; i < MAX_DPB; i++)
                    if (i != cur_frame && dpb[i].ref == REF_SHORT && dpb[i].pic_num == pn) {
                        dpb[i].ref = REF_LONG; dpb[i].long_term_idx = lti;
                    }
            } else if (op == 4) {
                max_long_term_idx = a - 1;
                for (int i = 0; i < MAX_DPB; i++)
                    if (i != cur_frame && dpb[i].ref == REF_LONG && dpb[i].long_term_idx > max_long_term_idx)
                        dpb[i].ref = REF_NONE;
            } else if (op == 5) {
                for (int i = 0; i < MAX_DPB; i++) if (i != cur_frame) dpb[i].ref = REF_NONE;
                max_long_term_idx = -1;
                cur_pic_has_mmco5 = 1;
            } else if (op == 6) {
                for (int i = 0; i < MAX_DPB; i++)
                    if (i != cur_frame && dpb[i].ref == REF_LONG && dpb[i].long_term_idx == a) dpb[i].ref = REF_NONE;
                cur->ref = REF_LONG; cur->long_term_idx = a; cur_long = 1;
            }
        }
    } else {
        int ns = 0, nl = 0;
        for (int i = 0; i < MAX_DPB; i++) {
            if (i == cur_frame) continue;
            if (dpb[i].ref == REF_SHORT) ns++;
            if (dpb[i].ref == REF_LONG) nl++;
        }
        if (ns + nl >= imax(sps->num_ref_frames, 1) && ns > 0) {
            int best = -1;
            for (int i = 0; i < MAX_DPB; i++) {
                if (i == cur_frame || dpb[i].ref != REF_SHORT) continue;
                int fnw = dpb[i].frame_num > sh->frame_num ? dpb[i].frame_num - max_frame_num : dpb[i].frame_num;
                dpb[i].frame_num_wrap = fnw;
                if (best < 0 || fnw < dpb[best].frame_num_wrap) best = i;
            }
            dpb[best].ref = REF_NONE;
        }
    }
    if (!cur_long) cur->ref = REF_SHORT;
    if (cur_pic_has_mmco5) cur->frame_num = 0;
}

/* ------------------------------------------------------------------ */
/* Picture management                                                 */
/* ------------------------------------------------------------------ */
static void output_frame(int idx)
{
    Frame *f = &dpb[idx];
    int cl = 0, cr = 0, ct = 0, cb = 0;
    if (!nocrop) { cl = sps->crop_left * 2; cr = sps->crop_right * 2; ct = sps->crop_top * 2; cb = sps->crop_bottom * 2; }
    for (int c = 0; c < 3; c++) {
        int sx = c ? 2 : 1;
        int x0 = cl / sx, x1 = (width - cr) / sx, y0 = ct / sx, y1 = (height - cb) / sx;
        for (int y = y0; y < y1; y++) fwrite(&f->plane[c][y * pw(c) + x0], 1, x1 - x0, fout);
    }
    frames_out++;
}

static void finish_picture(void)
{
    if (cur_frame < 0) return;
    if (mbs_decoded != mb_w * mb_h) fprintf(stderr, "warning: picture incomplete (%d/%d MBs)\n", mbs_decoded, mb_w * mb_h);
    for (int y = 0; y < mb_h; y++)
        for (int x = 0; x < mb_w; x++) deblock_mb(x, y);
    if (trace) {
        unsigned sum = 0;
        for (int c = 0; c < 3; c++)
            for (int i = 0; i < pw(c) * ph(c); i++) sum = sum * 31 + P(c)[i];
        fprintf(trace, "PICTURE done buf %d checksum %08x\n", cur_frame, sum);
    }
    mark_picture(&pic_sh);
    output_frame(cur_frame);
    dpb[cur_frame].in_use = (dpb[cur_frame].ref != REF_NONE);
    for (int i = 0; i < MAX_DPB; i++) if (dpb[i].ref == REF_NONE) dpb[i].in_use = 0;
    if (pic_sh.nal_ref_idc) prev_ref_frame_num = dpb[cur_frame].frame_num;
    cur_frame = -1;
}

static void start_picture(SliceHdr *sh)
{
    int new_w = sps->mb_width, new_h = sps->mb_height;
    if (new_w != mb_w || new_h != mb_h || !mbs) {
        mb_w = new_w; mb_h = new_h;
        width = mb_w * 16; height = mb_h * 16;
        free(mbs);
        mbs = calloc(mb_w * mb_h, sizeof(MbInfo));
        for (int i = 0; i < MAX_DPB; i++) {
            for (int c = 0; c < 3; c++) {
                free(dpb[i].plane[c]);
                dpb[i].plane[c] = calloc(1, pw(c) * ph(c));
            }
            dpb[i].ref = REF_NONE; dpb[i].in_use = 0;
        }
    }
    cur_frame = -1;
    for (int i = 0; i < MAX_DPB; i++) if (!dpb[i].in_use && dpb[i].ref == REF_NONE) { cur_frame = i; break; }
    if (cur_frame < 0) die("no free frame buffer");
    dpb[cur_frame].in_use = 1;
    dpb[cur_frame].ref = REF_NONE;
    dpb[cur_frame].frame_num = sh->frame_num;
    for (int i = 0; i < mb_w * mb_h; i++) mbs[i].slice_num = -1;
    mbs_decoded = 0;
    pic_sh = *sh;
    if (trace) fprintf(trace, "PICTURE start buf %d frame_num %d idr %d\n", cur_frame, sh->frame_num, sh->idr);
}


/* parser token trace (matches tb_parse.vhd) */
static void ptrace_mb(int addr)
{
    if (!ptrace) return;
    MbInfo *m = &mbs[addr];
    int mbx = addr % mb_w, mby = addr / mb_w;
    int ht = m->type <= MB_IPCM ? m->type : (m->type == MB_PSKIP ? 4 : 3);
    int cur_slice = m->slice_num;
    int qpc = chroma_qp_tab[clip3(0, 51, m->qp + pps->chroma_qp_index_offset)];
    fprintf(ptrace, "T %d t=%d qp=%d qpc=%d", addr, ht, m->qp, qpc);
    if (m->type == MB_I4x4) {
        fprintf(ptrace, " m=");
        for (int i = 0; i < 16; i++) fprintf(ptrace, "%x", m->i4mode[i]);
    }
    if (m->type == MB_I16x16) fprintf(ptrace, " m=%d", m->i16_mode);
    if (m->type == MB_I4x4 || m->type == MB_I16x16) {
        fprintf(ptrace, " c=%d av=%d%d%d%d", m->chroma_mode,
                intra_avail(mbx - 1, mby, cur_slice), intra_avail(mbx, mby - 1, cur_slice),
                intra_avail(mbx + 1, mby - 1, cur_slice), intra_avail(mbx - 1, mby - 1, cur_slice));
    }
    if (!IS_INTRA(m->type)) {
        fprintf(ptrace, " p=%d s=%d%d%d%d mv=", m->type == MB_PSKIP ? 0 : m->part,
                m->sub[0], m->sub[1], m->sub[2], m->sub[3]);
        for (int i = 0; i < 16; i++) fprintf(ptrace, "%d,%d;", m->mv[i][0], m->mv[i][1]);
        fprintf(ptrace, " rp=%d,%d,%d,%d", m->ref_pic[0], m->ref_pic[2], m->ref_pic[8], m->ref_pic[10]);
    }
    /* coefficient blocks present */
    unsigned nz = 0;
    if (m->type == MB_IPCM) nz = 0x3ff0000u & ~0x30000u, nz |= 0xffff;
    else {
        for (int i = 0; i < 16; i++) if (m->tc_luma[i]) nz |= 1u << i;
        if (m->type == MB_I16x16 && m->tc_ldc) nz |= 1u << 16;
        if ((m->cbp >> 4) & 3) nz |= 1u << 17;
        for (int c = 0; c < 2; c++) for (int b = 0; b < 4; b++) if (m->tc_chroma[c][b]) nz |= 1u << (18 + c * 4 + b);
    }
    fprintf(ptrace, " nz=%07x", nz);
    /* deblocking */
    int fl = mbx > 0 && m->disable_db != 1 && (m->disable_db != 2 || mbs[addr - 1].slice_num == m->slice_num);
    int ft = mby > 0 && m->disable_db != 1 && (m->disable_db != 2 || mbs[addr - mb_w].slice_num == m->slice_num);
    fprintf(ptrace, " f=%d%d%d qdb=%d", fl, ft, m->disable_db != 1, m->qp_db);
    if (fl) fprintf(ptrace, " ql=%d", mbs[addr - 1].qp_db);
    if (ft) fprintf(ptrace, " qt=%d", mbs[addr - mb_w].qp_db);
    if (m->disable_db != 1) {
        fprintf(ptrace, " bs=");
        for (int dir = 0; dir < 2; dir++)
            for (int e = 0; e < 4; e++)
                for (int k = 0; k < 4; k++) {
                    if (e == 0 && !(dir == 0 ? fl : ft)) { fprintf(ptrace, "x"); continue; }
                    MbInfo *p = m;
                    int qb = dir == 0 ? k * 4 + e : e * 4 + k, pb;
                    if (e == 0) { p = dir == 0 ? &mbs[addr - 1] : &mbs[addr - mb_w]; pb = dir == 0 ? k * 4 + 3 : 12 + k; }
                    else pb = dir == 0 ? qb - 1 : qb - 4;
                    fprintf(ptrace, "%d", bs_compute(p, pb, m, qb, e == 0));
                }
    }
    fprintf(ptrace, "\n");
    /* coefficient blocks */
    for (int b = 0; b < 26; b++) {
        if (!(nz & (1u << b))) continue;
        fprintf(ptrace, "C %d %d", addr, b);
        for (int i = 0; i < 16; i++) {
            int v = 0;
            if (m->type == MB_IPCM) {
                if (b < 16) v = m->pcm[((b / 4) * 4 + i / 4) * 16 + (b % 4) * 4 + i % 4];
                else { int c = (b - 18) / 4, bb = (b - 18) % 4;
                       v = m->pcm[256 + c * 64 + ((bb / 2) * 4 + i / 4) * 8 + (bb % 2) * 4 + i % 4]; }
            } else if (b < 16) v = m->coef_luma[b][i];
            else if (b == 16) v = m->coef_ldc[i];
            else if (b == 17) v = i < 4 ? m->coef_cdc[0][i] : i < 8 ? m->coef_cdc[1][i - 4] : 0;
            else v = m->coef_cac[(b - 18) / 4][(b - 18) % 4][i];
            fprintf(ptrace, " %d", v);
        }
        fprintf(ptrace, "\n");
    }
}

static void decode_slice(BitReader *br, int nal_type, int nal_ref_idc)
{
    SliceHdr sh;
    memset(&sh, 0, sizeof(sh));
    sh.first_mb = br_ue(br);
    sh.slice_type = br_ue(br) % 5;
    if (sh.slice_type != 0 && sh.slice_type != 2) die("unsupported slice type %d", sh.slice_type);
    sh.pps_id = br_ue(br);
    if (!pps_tab[sh.pps_id].valid) die("missing pps");
    PPS *np = &pps_tab[sh.pps_id];
    if (np->entropy_coding_mode) die("CABAC not supported");
    if (np->weighted_pred && sh.slice_type == 0) die("weighted prediction not supported");
    if (np->transform_8x8_mode) die("8x8 transform not supported");
    if (!sps_tab[np->sps_id].valid) die("missing sps");
    SPS *ns = &sps_tab[np->sps_id];
    sh.idr = (nal_type == 5);
    sh.nal_ref_idc = nal_ref_idc;
    sh.frame_num = br_u(br, ns->log2_max_frame_num);
    if (sh.first_mb == 0) {
        finish_picture();
        pps = np; sps = ns;
        start_picture(&sh);
    }
    pps = np;
    if (sh.idr) br_ue(br); /* idr_pic_id */
    if (ns->poc_type == 0) {
        br_u(br, ns->log2_max_poc_lsb);
        if (np->bottom_field_pic_order_in_frame_present) br_se(br);
    }
    if (ns->poc_type == 1 && !ns->delta_pic_order_always_zero) {
        br_se(br);
        if (np->bottom_field_pic_order_in_frame_present) br_se(br);
    }
    if (np->redundant_pic_cnt_present) { if (br_ue(br)) die("redundant pictures not supported"); }
    sh.num_ref_idx_active = np->num_ref_idx_l0_default;
    if (sh.slice_type == 0) {
        if (br_u1(br)) sh.num_ref_idx_active = br_ue(br) + 1;
        if (sh.num_ref_idx_active > 16) die("num_ref_idx");
        build_ref_list(&sh, br);
    }
    if (nal_ref_idc) parse_dec_ref_pic_marking(br, &sh);
    if (sh.first_mb == 0) pic_sh = sh;
    sh.qp = np->pic_init_qp + br_se(br);
    sh.disable_db = 0; sh.alpha_off = 0; sh.beta_off = 0;
    if (np->deblocking_filter_control_present) {
        sh.disable_db = br_ue(br);
        if (sh.disable_db != 1) {
            sh.alpha_off = br_se(br) * 2;
            sh.beta_off = br_se(br) * 2;
        }
    }
    if (trace) fprintf(trace, "SLICE first_mb %d type %d qp %d bit %d\n", sh.first_mb, sh.slice_type, sh.qp, br->pos);

    /* slice data */
    int qp = sh.qp;
    int addr = sh.first_mb;
    int more = 1;
    slice_num++;
    while (more) {
        if (addr >= mb_w * mb_h) die("mb address overflow");
        if (sh.slice_type == 0) {
            int run = br_ue(br);
            for (int i = 0; i < run; i++) {
                if (addr >= mb_w * mb_h) die("skip run overflow");
                int mbx = addr % mb_w, mby = addr / mb_w;
                MbInfo *m = &mbs[addr];
                m->slice_num = slice_num;
                m->disable_db = sh.disable_db; m->alpha_off = sh.alpha_off; m->beta_off = sh.beta_off;
                decode_mb_pskip(mbx, mby, &sh, qp);
                trace_mb(addr, m, br->pos);
                m->qp_db = m->qp;
                m->qpc_db = chroma_qp_tab[clip3(0, 51, m->qp + pps->chroma_qp_index_offset)];
                ptrace_mb(addr);
                recon_mb(mbx, mby);
                mbs_decoded++;
                addr++;
            }
            if (run > 0) more = br_more_rbsp_data(br);
            if (!more) break;
        }
        int mbx = addr % mb_w, mby = addr / mb_w;
        MbInfo *m = &mbs[addr];
        m->slice_num = slice_num;
        m->disable_db = sh.disable_db; m->alpha_off = sh.alpha_off; m->beta_off = sh.beta_off;
        decode_mb(br, mbx, mby, &sh, &qp);
        m->qp_db = m->type == MB_IPCM ? 0 : m->qp;
        m->qpc_db = chroma_qp_tab[clip3(0, 51, m->qp_db + pps->chroma_qp_index_offset)];
        ptrace_mb(addr);
        recon_mb(mbx, mby);
        mbs_decoded++;
        addr++;
        more = br_more_rbsp_data(br);
    }
}

/* ------------------------------------------------------------------ */
/* NAL parsing                                                        */
/* ------------------------------------------------------------------ */
static void decode_nal(const uint8_t *nal, int size)
{
    if (size < 1) return;
    int nal_ref_idc = (nal[0] >> 5) & 3;
    int nal_type = nal[0] & 31;
    /* remove emulation prevention */
    uint8_t *rbsp = malloc(size);
    int n = 0, zeros = 0;
    for (int i = 1; i < size; i++) {
        if (zeros >= 2 && nal[i] == 3) { zeros = 0; continue; }
        rbsp[n++] = nal[i];
        zeros = nal[i] == 0 ? zeros + 1 : 0;
    }
    BitReader br;
    br_init(&br, rbsp, n);
    if (trace) fprintf(trace, "NAL type %d ref %d size %d\n", nal_type, nal_ref_idc, n);
    switch (nal_type) {
    case 7: parse_sps(&br); break;
    case 8: parse_pps(&br); break;
    case 1: case 5: decode_slice(&br, nal_type, nal_ref_idc); break;
    default: break;
    }
    free(rbsp);
}

int main(int argc, char **argv)
{
    const char *in = NULL, *out = NULL, *tr = NULL;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-t") && i + 1 < argc) tr = argv[++i];
        else if (!strcmp(argv[i], "-p") && i + 1 < argc) ptrace = fopen(argv[++i], "w");
        else if (!strcmp(argv[i], "-r") && i + 1 < argc) rtrace = fopen(argv[++i], "w");
        else if (!strcmp(argv[i], "-nocrop")) nocrop = 1;
        else if (!in) in = argv[i];
        else out = argv[i];
    }
    if (!in || !out) { fprintf(stderr, "usage: %s in.264 out.yuv [-t trace] [-nocrop]\n", argv[0]); return 1; }
    FILE *f = fopen(in, "rb");
    if (!f) die("cannot open %s", in);
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t *buf = malloc(sz);
    if (fread(buf, 1, sz, f) != (size_t)sz) die("read");
    fclose(f);
    fout = fopen(out, "wb");
    if (!fout) die("cannot open %s", out);
    if (tr) trace = fopen(tr, "w");

    /* split Annex B */
    long i = 0, start = -1;
    while (i + 2 < sz) {
        if (buf[i] == 0 && buf[i + 1] == 0 && buf[i + 2] == 1) {
            if (start >= 0) {
                long end = i;
                while (end > start && buf[end - 1] == 0) end--;
                decode_nal(buf + start, end - start);
            }
            i += 3;
            start = i;
        } else i++;
    }
    if (start >= 0) {
        long end = sz;
        while (end > start && buf[end - 1] == 0) end--;
        decode_nal(buf + start, end - start);
    }
    finish_picture();
    fclose(fout);
    if (trace) fclose(trace);
    if (ptrace) fclose(ptrace);
    if (rtrace) fclose(rtrace);
    fprintf(stderr, "decoded %d frames (%dx%d)\n", frames_out, width, height);
    return 0;
}
