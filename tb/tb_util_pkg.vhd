-- ============================================================================
-- tb_util_pkg.vhd  --  simulation helpers (file input, formatting)
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;

package tb_util_pkg is
    type char_file_t is file of character;

    function itoa(v : integer) return string;
    function hex(v : std_logic_vector; digits : natural) return string;
    procedure wr(variable l : inout line; s : string);
end package;

package body tb_util_pkg is
    function itoa(v : integer) return string is
    begin
        return integer'image(v);
    end function;

    function hex(v : std_logic_vector; digits : natural) return string is
        constant HX : string(1 to 16) := "0123456789abcdef";
        variable r  : string(1 to digits);
        variable x  : unsigned(digits * 4 - 1 downto 0);
        variable d  : natural;
    begin
        x := resize(unsigned(v), digits * 4);
        for i in 0 to digits - 1 loop
            d := to_integer(x((digits - i) * 4 - 1 downto (digits - i - 1) * 4));
            r(i + 1) := HX(d + 1);
        end loop;
        return r;
    end function;

    procedure wr(variable l : inout line; s : string) is
    begin
        write(l, s);
    end procedure;
end package body;
