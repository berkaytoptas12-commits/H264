# PolarFire SoC için VHDL H.264 Çözücü (yalnızca FPGA kaynakları)

Microchip **PolarFire SoC MPFS250 Video Kit** üzerinde, RISC-V çekirdekleri (MSS) kullanılmadan,
tamamen FPGA dokusunda (fabric) çalışmak üzere yazılmış bir **H.264 / AVC Constrained Baseline**
çözücüsü. RTL dili **VHDL-2008**'dir.

* Giriş: AXI4-Stream bayt akışı (Annex-B, `00 00 01` başlangıç kodlu ham `.264`)
* Çıkış: Çözülen çerçeveler AXI4 üzerinden harici DDR'daki çerçeve tamponlarına yazılır
  (planar YUV 4:2:0) ve çözme sırasıyla bir çerçeve tanıtım arayüzünden bildirilir.
* Bit-exact: Üretilen tüm test akışlarında çıktı, **ffmpeg** çıktısıyla bayt bayt aynıdır
  (aşağıdaki doğrulama tablosuna bakın).

## 1. Desteklenen özellikler

| Özellik | Durum |
|---|---|
| Profil | Constrained Baseline (profile_idc 66; 77/100 SPS'leri de ayrıştırılır, CABAC vb. içermiyorlarsa çözülür) |
| Slice türleri | I, P (SP/SI/B yok) |
| Entropi | CAVLC (tüm coeff_token / level / total_zeros / run_before tabloları) |
| Makroblok türleri | I_NxN (Intra4x4, 9 mod), I_16x16 (4 mod), I_PCM, P_L0 16x16 / 16x8 / 8x16 / 8x8 (8x8, 8x4, 4x8, 4x4 alt bölütler), P_8x8ref0, P_Skip |
| Hareket kompanzasyonu | Çeyrek piksel luma (6-tap), 1/8 piksel kroma (bilineer), çerçeve dışına taşan vektörler (kenar tekrarı) |
| Referanslar | Çoklu referans (num_ref_frames ≤ 16), kısa ve uzun dönem referanslar, sliding window, MMCO 1–6, ref_pic_list_modification |
| Slice | Çoklu slice, `slice-max-size`, slice sınırında komşu erişilebilirliği |
| Deblocking | Tam döngü filtresi; `disable_deblocking_filter_idc` 0/1/2, alpha/beta ofsetleri, chroma_qp_index_offset |
| Diğer | constrained_intra_pred, POC tip 0/1/2 (çıkış sırası = çözme sırası), kırpma (cropping), tek/çok SPS-PPS (SPS id 0–31, PPS id 0–255) |
| Çözünürlük | 16x16'dan 2048x1152'ye kadar (1920x1080 dahil) |

Desteklenmeyenler (akışta görülürse `o_error` kalkar ve ilgili slice atlanır): CABAC, B slice,
FMO/ASO, interlace (field/MBAFF), 8x8 dönüşüm, ölçekleme matrisleri, ağırlıklı tahmin,
lossless, redundant picture, frame_num boşlukları (gaps_in_frame_num).

## 2. Mimari

Makroblok (MB) seviyesinde boru hattı; her aşama bir "MB token" (`mbinfo_t`) alır, işler ve bir
sonrakine verir. Aşamalar arası veriler çift/dört yuvalı tamponlardan geçer, böylece aşamalar farklı
MB'ler üzerinde paralel çalışır.

```
 AXI4-Stream ──► h264_nal ──► h264_bitreader ──► h264_hdr ──┐ (slice parametreleri, DPB)
 (Annex-B)     start code,    72 bit pencere      SPS/PPS/   │
               EPB silme                          slice hdr  ▼
                                               h264_mbparse ──► katsayı tamponu (4 yuva)
                                               MB katmanı, CAVLC,     │
                                               intra mod / MV tahmini,│
                                               bS hesabı              │
                                                    │ token           │
                                                    ▼                 │
            AXI4 okuma ◄──────────────────── h264_mc                  │
            (referans pencere)               6-tap / bilineer ─► tahmin tamponu (2 yuva)
                                                    │ token           │
                                                    ▼                 ▼
                                             h264_recon  ◄────────────┘
                                             ters niceleme, IDCT, Hadamard,
                                             intra 4x4/16x16/kroma tahmini
                                                    │ token ─► yeniden yapılandırılmış MB tamponu
                                                    ▼
            AXI4 yazma ◄──────────────────── h264_deblock
            (çerçeve tamponu)                döngü filtresi, (-4,-4) kaydırılmış çıkış penceresi
```

| Dosya | İçerik |
|---|---|
| `rtl/h264_pkg.vhd` | Ortak tipler, MB token kaydı (`mbinfo_t`), slice kaydı, Exp-Golomb, çerçeve adres fonksiyonu |
| `rtl/h264_tables_pkg.vhd` | **Üretilmiş** tablolar (VLC ROM'ları, CBP, zigzag, niceleme, alpha/beta/tC0). `scripts/gen_tables.py` |
| `rtl/h264_nal.vhd` | Start code bulma, emulation prevention baytı silme, NAL ilk/son bayt işaretleri |
| `rtl/h264_bitreader.vhd` | 72 bitlik tampon, 64 bit pencere, `more_rbsp_data()`, kayıtlı tüketim |
| `rtl/h264_hdr.vhd` | SPS/PPS tabloları (blok RAM), slice header, çerçeve tamponu tahsisi, referans işaretleme (sliding window, MMCO), referans listesi kurulumu ve modifikasyonu, çerçeve çıkış kuyruğu |
| `rtl/h264_mbparse.vhd` | mb_skip_run, mb_type, Intra4x4 mod tahmini, alt bölütler, ref_idx, MVD + MV tahmini (medyan / yönlü / P_Skip), CBP, QP, CAVLC artıkları, I_PCM, komşu satır tamponu, deblocking bS / QP / filtre bayrakları |
| `rtl/h264_mc.vhd` | Bölüt listesi, satır başına bir AXI okuma patlaması, kenar kırpma, luma 6-tap çeyrek piksel ve kroma 1/8 piksel enterpolasyon (satır akışlı, 16 piksel/çevrim) |
| `rtl/h264_recon.vhd` | DC Hadamard'lar, ters niceleme, 4x4 tamsayı ters dönüşüm, intra tahminler, PCM, toplama/kırpma |
| `rtl/h264_deblock.vhd` | 20x20 luma / 12x12 kroma çalışma dizisi, satır-sütun filtre boru hattı (kenar başına bir kademe), satır tamponları, bayt maskeli AXI yazma |
| `rtl/h264_ram.vhd` | Basit çift portlu RAM (LSRAM/uSRAM çıkarımı), 16 bankalı piksel tamponu |
| `rtl/h264_dec_top.vhd` | Üst seviye: tüm aşamalar + tek AXI4 master |

Deblocking çıkışı neden kaydırılmış pencere? Bir MB filtrelendiğinde, sağındaki ve altındaki MB'ler
henüz sınır satırlarını/sütunlarını değiştirecektir. Bu nedenle her MB adımında, kesinleşmiş olan
(-4,-4) kaydırılmış 16x16 bölge (kromada 8x8) yazılır; resmin sağ/alt kenarında bölge büyütülür.
Kesinleşmemiş satırlar satır tamponunda, sütunlar çalışma dizisinde bekler. Böylece DDR'a her piksel
yalnızca bir kez, son hâliyle yazılır.

## 3. Üst seviye arayüz (`h264_dec_top`)

| Port | Yön | Açıklama |
|---|---|---|
| `clk`, `rst` | in | Tek saat alanı, senkron aktif-yüksek reset |
| `s_axis_tdata[7:0]`, `tvalid`, `tready`, `tlast` | in/out | Annex-B bayt akışı; `tlast` akışın son baytında 1 |
| `m_axi_*` | master | AXI4, 32 bit adres, 64 bit veri, ID=0. Okuma: MC referans satırları (1–4 beat INCR). Yazma: deblock çıkış satırları (1–3 beat, `wstrb` ile) |
| `o_frame_valid`, `o_frame_index[4:0]`, `o_frame_addr_y/cb/cr`, `i_frame_ready` | out/in | Tamamen yazılmış bir çerçeve hazır (çözme sırasıyla). `valid & ready` el sıkışmasından sonra tampon tüketicinindir |
| `i_release`, `i_release_index[4:0]` | in | Tüketici (ör. ekran okuyucu) işi bitince tamponu geri verir (1 çevrim darbe) |
| `o_width`, `o_height`, `o_crop_*` | out | Etkin SPS'in kodlanmış boyutu ve kırpma değerleri (luma piksel) |
| `o_error`, `o_stream_done`, `o_pic_count` | out | Durum: desteklenmeyen/bozuk akış (yapışkan), akış sonu ve tüm çerçeveler teslim edildi, çözülen resim sayısı |

Generic: `G_FB_BASE` – çerçeve deposunun DDR'daki taban adresi (4 MiB hizalı).

## 4. Çerçeve deposu düzeni (DDR)

18 çerçeve tamponu (DPB ≤ 16 referans + çözülen + ekranda tutulan), her biri 4 MiB:

```
çerçeve i : G_FB_BASE + i * 0x400000
  Y  : +0x000000, satır adımı 2048 bayt
  Cb : +0x240000, satır adımı 1024 bayt
  Cr : +0x2D0000, satır adımı 1024 bayt
```

Toplam 72 MiB. Kodlanmış boyut 16'nın katıdır (1080p için 1920x1088); geçerli görüntü
`o_crop_*` ile verilir.

## 5. Doğrulama

Üç katmanlı doğrulama yapılır:

1. **C referans modeli** (`model/h264_model.c`): donanımla aynı bölümlemeyle yazılmış bit-exact model;
   ffmpeg ile karşılaştırılır.
2. **Ayrıştırıcı testbench'i** (`tb/tb_parse.vhd`): NAL → bitreader → hdr → mbparse; her MB token'ı ve
   katsayı blokları C modelinin `-p` izi ile satır satır karşılaştırılır.
3. **Tam çözücü testbench'i** (`tb/tb_top.vhd`): AXI4 bellek modeli (okuma gecikmesi, çoklu bekleyen
   istek, rastgele geri basınç) ile tüm çözücü; çıkan YUV, ffmpeg çıktısıyla karşılaştırılır.

Test akışları x264 ile üretilir (`scripts/gen_streams.sh`): intra-only, IP, 3–4 referans, tüm bölüt
türleri, gürültülü içerik, çoklu slice, `slice-max-size`, deblocking kapalı / ofsetli, constrained
intra + intra refresh, QP 1/10/51, chroma QP ofseti ±, AQ (mb_qp_delta), 16'nın katı olmayan boyutlar
(kırpma), büyük hareket vektörleri ve çerçeve dışı referanslar, 720p, 1080p.

Sonuçlar (GHDL 4.1, `STALL=1` rastgele AXI geri basıncı, okuma gecikmesi 12 çevrim):

| Akış | Boyut | Çerçeve | C modeli | RTL (tb_top) | Çevrim/MB |
|---|---|---|---|---|---|
| qcif_intra | 176x144 | 3 | bit-exact | bit-exact | 293 |
| qcif_ip | 176x144 | 8 | bit-exact | bit-exact | 300 |
| qcif_ref3 (tüm bölütler) | 176x144 | 10 | bit-exact | bit-exact | 302 |
| qcif_noise (QP 20, gürültü) | 176x144 | 6 | bit-exact | bit-exact | 1417 |
| qcif_slices / slmax | 176x144 | 6 | bit-exact | bit-exact | 303 / 923 |
| qcif_cip (constrained intra) | 176x144 | 8 | bit-exact | bit-exact | 312 |
| qcif_nodb / dbofs / dbneg | 176x144 | 6 | bit-exact | bit-exact | 300 |
| qcif_qp1 / qp10 / qp51 | 176x144 | 3/4/4 | bit-exact | bit-exact | 322 / 1453 / 303 |
| qcif_cqp / cqpn (chroma QP ofseti, AQ) | 176x144 | 6 | bit-exact | bit-exact | 299 |
| odd_200x120 / odd_100x60 | kırpmalı | 6 | bit-exact | bit-exact | 298 / 331 |
| cif_ref4 / cif_bigmv / cif_gop | 352x288 | 10/8/20 | bit-exact | bit-exact | 298 / 289 / 296 |
| hd720 / hd1080 | 1280x720 / 1920x1080 | 3 / 2 | bit-exact | bkz. not | – |

Not: tabloda RTL sonucu henüz yazılmamış satırlar için ayrıştırıcı testbench'i (tb_parse) bit-exact'tir;
tam çözücü sonuçları README güncellendikçe eklenir.

### Çalıştırma

Gerekenler: GHDL ≥ 4 (`--std=08`), gcc, python3, test akışı üretmek için libx264'lü ffmpeg.

```sh
scripts/gen_streams.sh streams          # x264 akışları + ffmpeg referans YUV'ları (quick: yalnızca QCIF)
scripts/run_sim.sh model streams        # C modeli ↔ ffmpeg
scripts/run_sim.sh parse streams        # RTL ayrıştırıcı ↔ C modeli izi
scripts/run_sim.sh top   streams qcif_ip qcif_ref3   # tam RTL ↔ ffmpeg
```

`scripts/gen_tables.py`, C modelindeki standart tablolarından `rtl/h264_tables_pkg.vhd`'yi üretir
ve VLC ROM'larını olası tüm 16 bitlik desenlerle doğrular.

## 6. Performans

Ölçülen ortalama çevrim/MB değerleri yukarıdaki tablodadır. Tipik akışlarda ~300 çevrim/MB,
çok yüksek bit hızında (QP ≤ 20, gürültülü içerik) CAVLC'ye bağlı olarak ~1400 çevrim/MB.

| Hedef | Gerekli MB/s | 300 çevrim/MB ile gereken saat |
|---|---|---|
| 1280x720 @ 30 | 108 000 | 33 MHz |
| 1920x1080 @ 30 | 244 800 | 74 MHz |
| 1920x1080 @ 60 | 489 600 | 147 MHz |

Kritik yolları kısa tutmak için bit okuyucunun tüketim girişi kayıtlıdır (ayrıştırıcı sembol başına 2
çevrim). PolarFire'da 100–150 MHz hedeflenir; zamanlama kapanışı Libero'da doğrulanmalıdır.

## 7. MPFS250 Video Kit'e entegrasyon (Libero SoC)

1. `rtl/*.vhd` dosyalarını bir HDL kütüphanesine (work) ekleyin; dil seçeneği VHDL-2008.
   `h264_dec_top` bir SmartDesign bileşeni olarak eklenir.
2. Saat: PF_CCC'den tek bir saat (ör. 125–150 MHz). Reset senkron ve aktif-yüksektir
   (CORERESET_PF çıkışından bu saat alanına senkronlanmış).
3. Bellek: `m_axi_*` portu çerçeve deposuna bağlanır. Kitte DDR'ın nasıl bağlı olduğuna göre:
   * fabric tarafı DDR4 varsa **PF_DDR4** IP'sinin AXI4 slave portuna (veri genişliği farklıysa
     araya **CoreAXI4Interconnect** ile genişlik dönüşümü),
   * aksi hâlde aynı interconnect üzerinden ekran/kamera yolunun kullandığı bellek denetleyicisine.
   `G_FB_BASE` bu bellek penceresinde 72 MiB'lık boş bir alanı göstermelidir.
4. Bit akışı kaynağı: `s_axis_*` arayüzüne herhangi bir fabric kaynağı bağlanabilir (ör. DDR'dan
   okuyan basit bir DMA, Ethernet/UDP alıcısı, SD kart okuyucu). Akışın son baytında `tlast=1`.
5. Ekran: `o_frame_valid/index` ile gelen tamponu, kitin HDMI/ekran yolunu besleyen bir çerçeve
   okuyucusu (YUV420 → RGB/YUV422 dönüşümü ile) okur; okuma bitince `i_release` ile geri verir.
   Çıkış çerçeve hızını ayarlamak (ör. 30 fps) tüketicinin görevidir; tüketici tamponu tutarken
   çözücü yeni tamponu kullanamaz, en fazla 18 tampon döngüdedir.
6. `o_error` ve `o_pic_count` hata ayıklama için bir LED'e / SmartDebug'a bağlanabilir.

Tahmini bellek kullanımı: satır tamponları + MB tamponları + SPS/PPS tabloları ≈ 250 Kbit
(birkaç düzine LSRAM; MPFS250'de 812 LSRAM vardır). Mantık kullanımı Libero sentez raporundan
alınmalıdır.

## 8. Dizin yapısı

```
model/    C referans modeli (h264_model.c)
rtl/      VHDL-2008 RTL
tb/       GHDL testbench'leri (tb_parse, tb_top, tb_util_pkg)
scripts/  gen_streams.sh, gen_tables.py, run_sim.sh, yuvcmp.py
```

## 9. Bilinen sınırlar / yol haritası

* Main/High profil (CABAC, B slice, 8x8 dönüşüm) desteklenmiyor.
* Çıkış sırası çözme sırasıdır; B slice olmayan akışlarda bu, gösterim sırasıyla aynıdır.
* Hata gizleme (error concealment) yok; eksik slice'lı bir resim tamamlanmaz.
* Performans iyileştirmeleri: recon'da blok boru hattı, MC'de MB'ler arası örtüşme, CAVLC'de
  sembol başına tek çevrim.
