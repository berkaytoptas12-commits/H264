-- ============================================================================
-- h264_pkg.vhd
-- Common types, constants and helper functions of the H.264 Constrained
-- Baseline decoder.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package h264_pkg is

    -- ------------------------------------------------------------------
    -- Limits
    -- ------------------------------------------------------------------
    constant C_MAX_MB_W   : natural := 128;     -- 2048 luma samples
    constant C_MAX_MB_H   : natural := 72;      -- 1152 luma samples
    constant C_NUM_FB     : natural := 18;      -- frame buffers (DPB + display)
    constant C_FB_BITS    : natural := 5;

    -- ------------------------------------------------------------------
    -- Frame buffer memory layout (planar 4:2:0, byte addressed)
    --   frame i at FB_BASE + i * 4 MiB
    --   Y  at +0x000000, stride 2048
    --   Cb at +0x240000, stride 1024
    --   Cr at +0x2D0000, stride 1024
    -- ------------------------------------------------------------------
    constant C_FB_SIZE_LOG2   : natural := 22;
    constant C_Y_STRIDE_LOG2  : natural := 11;
    constant C_C_STRIDE_LOG2  : natural := 10;
    constant C_CB_OFFSET      : natural := 16#240000#;
    constant C_CR_OFFSET      : natural := 16#2D0000#;

    function fb_plane_addr(base : unsigned(31 downto 0); fb : unsigned; plane : natural range 0 to 2;
                           x : unsigned; y : unsigned) return unsigned;

    -- ------------------------------------------------------------------
    -- Small helper types
    -- ------------------------------------------------------------------
    subtype u4_t  is unsigned(3 downto 0);
    subtype u5_t  is unsigned(4 downto 0);
    subtype u6_t  is unsigned(5 downto 0);
    subtype u8_t  is unsigned(7 downto 0);
    subtype s16_t is signed(15 downto 0);
    type u4_arr16_t  is array (0 to 15) of u4_t;
    type u5_arr4_t   is array (0 to 3) of u5_t;
    type u5_arr16_t  is array (0 to 15) of u5_t;
    type s16_arr16_t is array (0 to 15) of s16_t;
    type u3_arr32_t  is array (0 to 31) of unsigned(2 downto 0);
    type u2_arr4_t   is array (0 to 3) of unsigned(1 downto 0);
    type mv_t is record
        x : s16_t;
        y : s16_t;
    end record;
    type mv_arr16_t is array (0 to 15) of mv_t;
    constant MV_ZERO : mv_t := (x => (others => '0'), y => (others => '0'));

    -- macroblock classes
    constant MBT_I4x4  : unsigned(2 downto 0) := "000";
    constant MBT_I16   : unsigned(2 downto 0) := "001";
    constant MBT_IPCM  : unsigned(2 downto 0) := "010";
    constant MBT_P     : unsigned(2 downto 0) := "011";
    constant MBT_PSKIP : unsigned(2 downto 0) := "100";
    function mbt_is_intra(t : unsigned(2 downto 0)) return boolean;

    -- coefficient buffer block numbers (4 rows of 4 x 16 bit per block)
    --   0..15  luma 4x4 blocks in raster order
    --   16     Intra16x16 DC (raster order of the 4x4 DC matrix)
    --   17     chroma DC: row 0 = Cb (c0..c3), row 1 = Cr
    --   18..21 Cb AC blocks (raster), 22..25 Cr AC blocks
    -- for I_PCM luma samples live in blocks 0..15 and chroma in 18..25
    constant CB_LDC : natural := 16;
    constant CB_CDC : natural := 17;
    constant CB_CB  : natural := 18;
    constant CB_CR  : natural := 22;
    constant C_COEF_SLOTS : natural := 4;

    -- ------------------------------------------------------------------
    -- Macroblock information token passed down the pipeline
    -- ------------------------------------------------------------------
    type mbinfo_t is record
        -- position / picture
        mb_x          : unsigned(6 downto 0);
        mb_y          : unsigned(6 downto 0);
        pic_w         : unsigned(7 downto 0);   -- picture width in MBs
        pic_h         : unsigned(7 downto 0);   -- picture height in MBs
        first_of_pic  : std_logic;
        last_of_pic   : std_logic;
        cur_fb        : unsigned(4 downto 0);
        coef_slot     : unsigned(1 downto 0);
        pred_slot     : std_logic;
        rec_slot      : std_logic;
        -- type
        mbtype        : unsigned(2 downto 0);
        part          : unsigned(1 downto 0);   -- 0 16x16, 1 16x8, 2 8x16, 3 8x8
        sub           : u2_arr4_t;              -- 0 8x8, 1 8x4, 2 4x8, 3 4x4
        -- intra
        i16_mode      : unsigned(1 downto 0);
        chroma_mode   : unsigned(1 downto 0);
        i4_modes      : u4_arr16_t;             -- raster 4x4 order
        avail_a       : std_logic;              -- intra neighbour availability
        avail_b       : std_logic;
        avail_c       : std_logic;
        avail_d       : std_logic;
        -- residual
        qp            : unsigned(5 downto 0);
        qpc           : unsigned(5 downto 0);
        nz            : std_logic_vector(25 downto 0);  -- coef block present
        -- inter
        mv            : mv_arr16_t;             -- raster 4x4 order
        refpic        : u5_arr4_t;              -- dpb index per 8x8
        -- deblocking
        bs            : u3_arr32_t;             -- [dir*16 + edge*4 + k]
        qp_db         : unsigned(5 downto 0);
        qp_left       : unsigned(5 downto 0);
        qp_top        : unsigned(5 downto 0);
        qpc_db        : unsigned(5 downto 0);
        qpc_left      : unsigned(5 downto 0);
        qpc_top       : unsigned(5 downto 0);
        filt_left     : std_logic;
        filt_top      : std_logic;
        filt_int      : std_logic;
        alpha_off     : signed(4 downto 0);
        beta_off      : signed(4 downto 0);
    end record;

    constant MBINFO_INIT : mbinfo_t := (
        mb_x => (others => '0'), mb_y => (others => '0'),
        pic_w => (others => '0'), pic_h => (others => '0'),
        first_of_pic => '0', last_of_pic => '0',
        cur_fb => (others => '0'), coef_slot => (others => '0'),
        pred_slot => '0', rec_slot => '0',
        mbtype => (others => '0'), part => (others => '0'),
        sub => (others => (others => '0')),
        i16_mode => (others => '0'), chroma_mode => (others => '0'),
        i4_modes => (others => (others => '0')),
        avail_a => '0', avail_b => '0', avail_c => '0', avail_d => '0',
        qp => (others => '0'), qpc => (others => '0'), nz => (others => '0'),
        mv => (others => MV_ZERO), refpic => (others => (others => '0')),
        bs => (others => (others => '0')),
        qp_db => (others => '0'), qp_left => (others => '0'), qp_top => (others => '0'),
        qpc_db => (others => '0'), qpc_left => (others => '0'), qpc_top => (others => '0'),
        filt_left => '0', filt_top => '0', filt_int => '0',
        alpha_off => (others => '0'), beta_off => (others => '0'));

    -- ------------------------------------------------------------------
    -- Slice parameters handed from the header parser to the MB parser
    -- ------------------------------------------------------------------
    type slice_t is record
        first_mb      : unsigned(13 downto 0);
        is_p          : std_logic;
        qp            : unsigned(5 downto 0);
        num_ref       : unsigned(4 downto 0);   -- num_ref_idx_l0_active (1..16)
        ref_list      : u5_arr16_t;             -- dpb index per refIdx
        disable_db    : unsigned(1 downto 0);
        alpha_off     : signed(4 downto 0);
        beta_off      : signed(4 downto 0);
        chroma_qp_off : signed(4 downto 0);
        constr_intra  : std_logic;
        mb_w          : unsigned(7 downto 0);
        mb_h          : unsigned(7 downto 0);
        cur_fb        : unsigned(4 downto 0);
    end record;

    constant SLICE_INIT : slice_t := (
        first_mb => (others => '0'), is_p => '0', qp => (others => '0'),
        num_ref => (others => '0'), ref_list => (others => (others => '0')),
        disable_db => (others => '0'), alpha_off => (others => '0'),
        beta_off => (others => '0'), chroma_qp_off => (others => '0'),
        constr_intra => '0', mb_w => (others => '0'), mb_h => (others => '0'),
        cur_fb => (others => '0'));

    -- ------------------------------------------------------------------
    -- Bitstream helpers
    -- ------------------------------------------------------------------
    -- count leading zeros of a vector (result = length when all zero)
    function clz(v : std_logic_vector) return natural;

    -- Exp-Golomb on a left aligned window.  Codes up to 2*15+1 bits on a 32
    -- bit window, up to 2*31+1 on a 64 bit window.
    type eg_t is record
        val : unsigned(31 downto 0);     -- codeNum
        len : unsigned(6 downto 0);      -- code length in bits
        ok  : std_logic;                 -- code fits into the window
    end record;
    function ue_decode(w : std_logic_vector) return eg_t;
    function ue_decode_p1(w : std_logic_vector) return eg_t;    -- val = codeNum + 1
    function se_from_ue(k : unsigned(31 downto 0)) return signed;

    function clip_u8(v : integer) return unsigned;
    function clip3(lo, hi, v : integer) return integer;
    function to_sl(b : boolean) return std_logic;

end package h264_pkg;

package body h264_pkg is

    function fb_plane_addr(base : unsigned(31 downto 0); fb : unsigned; plane : natural range 0 to 2;
                           x : unsigned; y : unsigned) return unsigned is
        variable a : unsigned(31 downto 0);
    begin
        a := base + shift_left(resize(fb, 32), C_FB_SIZE_LOG2);
        if plane = 0 then
            a := a + shift_left(resize(y, 32), C_Y_STRIDE_LOG2) + resize(x, 32);
        elsif plane = 1 then
            a := a + to_unsigned(C_CB_OFFSET, 32) + shift_left(resize(y, 32), C_C_STRIDE_LOG2) + resize(x, 32);
        else
            a := a + to_unsigned(C_CR_OFFSET, 32) + shift_left(resize(y, 32), C_C_STRIDE_LOG2) + resize(x, 32);
        end if;
        return a;
    end function;

    function mbt_is_intra(t : unsigned(2 downto 0)) return boolean is
    begin
        return t = MBT_I4x4 or t = MBT_I16 or t = MBT_IPCM;
    end function;

    -- leading zero count (vectors up to 64 bits) as a binary search: log2 depth
    function clz(v : std_logic_vector) return natural is
        constant N : natural := v'length;
        variable x : std_logic_vector(63 downto 0) := (others => '0');
        variable cnt : natural range 0 to 64 := 0;
    begin
        x(63 downto 64 - N) := v;
        if x(63 downto 32) = x"00000000" then
            cnt := cnt + 32;
            x := x(31 downto 0) & x"00000000";
        end if;
        if x(63 downto 48) = x"0000" then
            cnt := cnt + 16;
            x := x(47 downto 0) & x"0000";
        end if;
        if x(63 downto 56) = x"00" then
            cnt := cnt + 8;
            x := x(55 downto 0) & x"00";
        end if;
        if x(63 downto 60) = "0000" then
            cnt := cnt + 4;
            x := x(59 downto 0) & "0000";
        end if;
        if x(63 downto 62) = "00" then
            cnt := cnt + 2;
            x := x(61 downto 0) & "00";
        end if;
        if x(63) = '0' then
            cnt := cnt + 1;
        end if;
        if cnt > N or unsigned(v) = 0 then
            cnt := N;
        end if;
        return cnt;
    end function;

    function ue_decode(w : std_logic_vector) return eg_t is
        constant N   : natural := w'length;
        variable wv  : std_logic_vector(N - 1 downto 0);
        variable lz  : natural;
        variable r   : eg_t;
        variable sh  : unsigned(N - 1 downto 0);
    begin
        wv := w;
        lz := clz(wv);
        r.val := (others => '0');
        r.len := (others => '0');
        r.ok := '0';
        if lz <= 31 and 2 * lz + 1 <= N then
            -- the 2*lz+1 code bits, right aligned, are codeNum + 1
            sh := shift_right(unsigned(wv), N - 1 - 2 * lz);
            r.val := resize(sh, 32) - 1;
            r.len := to_unsigned(2 * lz + 1, 7);
            r.ok := '1';
        end if;
        return r;
    end function;

    function ue_decode_p1(w : std_logic_vector) return eg_t is
        constant N   : natural := w'length;
        variable wv  : std_logic_vector(N - 1 downto 0);
        variable lz  : natural;
        variable r   : eg_t;
    begin
        wv := w;
        lz := clz(wv);
        r.val := (others => '0');
        r.len := (others => '0');
        r.ok := '0';
        if lz <= 31 and 2 * lz + 1 <= N then
            r.val := resize(shift_right(unsigned(wv), N - 1 - 2 * lz), 32);
            r.len := to_unsigned(2 * lz + 1, 7);
            r.ok := '1';
        end if;
        return r;
    end function;

    function se_from_ue(k : unsigned(31 downto 0)) return signed is
        variable r : signed(31 downto 0);
    begin
        if k(0) = '1' then
            r := signed(shift_right(k + 1, 1));
        else
            r := -signed(shift_right(k, 1));
        end if;
        return r;
    end function;

    function clip_u8(v : integer) return unsigned is
    begin
        if v < 0 then
            return to_unsigned(0, 8);
        elsif v > 255 then
            return to_unsigned(255, 8);
        else
            return to_unsigned(v, 8);
        end if;
    end function;

    function clip3(lo, hi, v : integer) return integer is
    begin
        if v < lo then
            return lo;
        elsif v > hi then
            return hi;
        end if;
        return v;
    end function;

    function to_sl(b : boolean) return std_logic is
    begin
        if b then
            return '1';
        end if;
        return '0';
    end function;

end package body h264_pkg;
