-- ============================================================================
-- h264_nal.vhd
-- Annex-B byte stream to NAL unit payload bytes.
--   * finds start codes (00 00 01), drops leading/trailing zero bytes
--   * removes emulation_prevention_three_byte (00 00 03 -> 00 00)
--   * flags the first byte (NAL header) and the last byte of every NAL unit
--   * input tlast marks the end of the stream; the last byte delivered after
--     it carries o_eos = '1' (a dummy NAL header 0x00 is sent if needed)
-- One input byte per cycle; up to one output byte per cycle.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity h264_nal is
    port (
        clk      : in  std_logic;
        rst      : in  std_logic;
        -- AXI4-Stream byte input
        s_tdata  : in  std_logic_vector(7 downto 0);
        s_tvalid : in  std_logic;
        s_tlast  : in  std_logic;
        s_tready : out std_logic;
        -- NAL byte output
        o_data   : out std_logic_vector(7 downto 0);
        o_first  : out std_logic;
        o_last   : out std_logic;
        o_eos    : out std_logic;
        o_valid  : out std_logic;
        i_ready  : in  std_logic
    );
end entity;

architecture rtl of h264_nal is
    signal zcnt        : unsigned(15 downto 0);
    signal in_nal      : std_logic;
    signal first_pend  : std_logic;
    -- held (last non-zero) byte of the current NAL
    signal held_v      : std_logic;
    signal held_d      : std_logic_vector(7 downto 0);
    signal held_f      : std_logic;
    -- pending emissions
    signal pend_held   : std_logic;
    signal pend_last   : std_logic;
    signal pend_z      : unsigned(15 downto 0);
    signal pend_new    : std_logic;
    signal new_d       : std_logic_vector(7 downto 0);
    signal flush       : std_logic;
    signal done        : std_logic;
    -- output register
    signal ov          : std_logic;
    signal od          : std_logic_vector(7 downto 0);
    signal of_r, ol_r, oe_r : std_logic;
    signal idle        : std_logic;
begin
    idle <= '1' when pend_held = '0' and pend_z = 0 and pend_new = '0' and flush = '0' and done = '0' else '0';
    s_tready <= idle;

    o_valid <= ov;
    o_data  <= od;
    o_first <= of_r;
    o_last  <= ol_r;
    o_eos   <= oe_r;

    process (clk)
        variable can_emit : boolean;
        variable b        : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            if rst = '1' then
                zcnt <= (others => '0');
                in_nal <= '0';
                first_pend <= '0';
                held_v <= '0';
                held_d <= (others => '0');
                held_f <= '0';
                pend_held <= '0';
                pend_last <= '0';
                pend_z <= (others => '0');
                pend_new <= '0';
                new_d <= (others => '0');
                flush <= '0';
                done <= '0';
                ov <= '0';
                od <= (others => '0');
                of_r <= '0';
                ol_r <= '0';
                oe_r <= '0';
            else
                if ov = '1' and i_ready = '1' then
                    ov <= '0';
                end if;
                can_emit := ov = '0' or i_ready = '1';

                if can_emit then
                    if pend_held = '1' then
                        ov <= '1';
                        od <= held_d;
                        of_r <= held_f;
                        ol_r <= pend_last;
                        oe_r <= '0';
                        pend_held <= '0';
                    elsif pend_z /= 0 then
                        ov <= '1';
                        od <= (others => '0');
                        of_r <= '0';
                        ol_r <= '0';
                        oe_r <= '0';
                        pend_z <= pend_z - 1;
                    elsif pend_new = '1' then
                        held_v <= '1';
                        held_d <= new_d;
                        held_f <= first_pend;
                        first_pend <= '0';
                        pend_new <= '0';
                    elsif flush = '1' then
                        -- end of stream: emit held byte as last + eos
                        ov <= '1';
                        if held_v = '1' then
                            od <= held_d;
                            of_r <= held_f;
                        else
                            od <= (others => '0');
                            of_r <= '1';
                        end if;
                        ol_r <= '1';
                        oe_r <= '1';
                        held_v <= '0';
                        flush <= '0';
                        done <= '1';
                    end if;
                end if;

                if idle = '1' and s_tvalid = '1' then
                    b := unsigned(s_tdata);
                    if b = 0 then
                        if zcnt /= x"FFFF" then
                            zcnt <= zcnt + 1;
                        end if;
                    elsif b = 3 and zcnt >= 2 and in_nal = '1' then
                        -- emulation prevention byte
                        pend_held <= held_v;
                        pend_last <= '0';
                        held_v <= '0';
                        pend_z <= zcnt;
                        zcnt <= (others => '0');
                    elsif b = 1 and zcnt >= 2 then
                        -- start code: the held byte ends the previous NAL
                        pend_held <= held_v;
                        pend_last <= '1';
                        held_v <= '0';
                        zcnt <= (others => '0');
                        in_nal <= '1';
                        first_pend <= '1';
                    else
                        if in_nal = '1' then
                            pend_held <= held_v;
                            pend_last <= '0';
                            held_v <= '0';
                            pend_z <= zcnt;
                            pend_new <= '1';
                            new_d <= s_tdata;
                        end if;
                        zcnt <= (others => '0');
                    end if;
                    if s_tlast = '1' then
                        flush <= '1';
                    end if;
                end if;
            end if;
        end if;
    end process;
end architecture;
