# PolarFire SoC için VHDL H.264 Çözücü (yalnızca FPGA kaynakları)

Microchip **PolarFire SoC MPFS250 Video Kit** üzerinde, RISC-V çekirdekleri (MSS) kullanılmadan,
tamamen FPGA dokusunda (fabric) çalışmak üzere yazılmış bir **H.264 / AVC Constrained Baseline**
çözücüsü. RTL dili **VHDL-2008**'dir.

* Giriş: AXI4-Stream bayt akışı (Annex-B, `00 00 01` başlangıç kodlu ham `.264`)
* Çıkış: Çözülen çerçeveler AXI4 üzerinden harici DDR'daki çerçeve tamponlarına yazılır
  (planar YUV 4:2:0) ve çözme sırasıyla bir çerçeve tanıtım arayüzünden bildirilir.
* Bit-exact: Üretilen tüm test akışlarında çıktı, **ffmpeg** çıktısıyla bayt bayt aynıdır
  (aşağıdaki doğrulama tablosuna bakın).
* PC'den video gönderme: **Ethernet** (bölüm 9; MSS'teki küçük bir bare-metal program yalnızca ağ
  paketlerini fabric'e taşır, çözme yine fabric'tedir). İsteğe bağlı seri hat alternatifi bölüm 8'dedir.
* Görüntü çıkışı: **HDMI** (bölüm 10; DDR'daki kareler fabric'te okunur, RGB'ye çevrilir, TMDS olarak
  transceiver hatlarından 1080p60 basılır).

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
| `rtl/h264_bitreader.vhd` | 72 bitlik tampon, 64 bit pencere, `more_rbsp_data()`; bayt ekleme kararı o çevrimin tüketiminden bağımsız (kısa yol) |
| `rtl/h264_hdr.vhd` | SPS/PPS tabloları (blok RAM), slice header, çerçeve tamponu tahsisi, referans işaretleme (sliding window, MMCO), referans listesi kurulumu ve modifikasyonu, çerçeve çıkış kuyruğu |
| `rtl/h264_mbparse.vhd` | mb_skip_run, mb_type, Intra4x4 mod tahmini, alt bölütler, ref_idx, MVD + MV tahmini (medyan / yönlü / P_Skip), CBP, QP, CAVLC artıkları, I_PCM, komşu satır tamponu, deblocking bS / QP / filtre bayrakları. Tüm VLC türleri kayıtlı bir çözme aşamasında paralel çözülür; tüketilecek bit sayısı da orada hesaplanır (sembol başına 2 çevrim) |
| `rtl/h264_mc.vhd` | Bölüt listesi, satır başına bir AXI okuma patlaması, kenar kırpma, luma 6-tap çeyrek piksel ve kroma 1/8 piksel enterpolasyon (satır akışlı, 16 piksel/çevrim) |
| `rtl/h264_recon.vhd` | DC Hadamard'lar, ters niceleme, 4x4 tamsayı ters dönüşüm, intra tahminler, PCM, toplama/kırpma; 4x4 blok başına bir çevrimlik 6 kademeli boru hattı (~40 çevrim/MB) |
| `rtl/h264_deblock.vhd` | 20x20 luma / 12x12 kroma çalışma dizisi, satır-sütun filtre boru hattı (kenar başına bir kademe), satır tamponları, bayt maskeli AXI yazma (adres kanalı veri kanalının önünde, beat'ler arka arkaya) |
| `rtl/h264_ram.vhd` | Basit çift portlu RAM (LSRAM/uSRAM çıkarımı, 2 çevrim okuma gecikmesi), 4 şeritli katsayı tamponu, tahmin tamponu |
| `rtl/h264_dec_top.vhd` | Üst seviye: tüm aşamalar + tek AXI4 master |
| `rtl/h264_uart.vhd` | 8N1 UART alıcı / verici (fabric) |
| `rtl/h264_link.vhd` | PC bağlantısı protokol motoru: CRC16'lı çerçeveler, ACK/NAK, kredi, 32 KB FIFO, START/END/STATUS (bölüm 8) |
| `rtl/h264_dec_uart_top.vhd` | Çözücü + UART + protokol motoru; kitte SmartDesign'a eklenecek blok |

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

x264'ün Baseline'da hiç üretmediği özellikler için `scripts/stream_tools.py` sentetik akışlar üretir:
x264 akışlarının SPS / PPS / slice başlıklarını yeniden yazar (slice verisi bitleri aynen kopyalanır)
ya da sıfırdan I_PCM akışı oluşturur. Bunlar da standarda uygundur; referans çıktıları yine
ffmpeg'den alınır (`syn_*` satırları).

Sonuçlar (GHDL 4.1, `STALL=1` rastgele AXI geri basıncı, okuma gecikmesi 12 çevrim):

| Akış | Boyut | Çerçeve | tb_parse | tb_top (RTL) | Çevrim/MB |
|---|---|---|---|---|---|
| qcif_intra | 176x144 | 3 | bit-exact | bit-exact | 220 |
| qcif_ip | 176x144 | 8 | bit-exact | bit-exact | 199 |
| qcif_ref3 (3 referans, tüm bölütler) | 176x144 | 10 | bit-exact | bit-exact | 200 |
| qcif_noise (QP 20, gürültü) | 176x144 | 6 | bit-exact | bit-exact | 1374 |
| qcif_slices / slmax | 176x144 | 6/6 | bit-exact | bit-exact | 201 / 918 |
| qcif_cip (constrained intra) | 176x144 | 8 | bit-exact | bit-exact | 200 |
| qcif_nodb / dbofs / dbneg | 176x144 | 6/6/6 | bit-exact | bit-exact | 201 / 201 / 201 |
| qcif_qp1 / qp10 / qp51 | 176x144 | 3/4/4 | bit-exact | bit-exact | 237 / 1415 / 199 |
| qcif_cqp / cqpn (chroma QP ofseti, AQ) | 176x144 | 6/6 | bit-exact | bit-exact | 199 / 201 |
| odd_200x120 / odd_100x60 | kırpmalı | 6/6 | bit-exact | bit-exact | 202 / 221 |
| cif_ref4 / cif_bigmv / cif_gop | 352x288 | 10/8/20 | bit-exact | bit-exact | 195 / 195 / 195 |
| hd720 / hd1080 | 1280x720 / 1920x1080 | 3/2 | bit-exact / – | bit-exact | 208 / 208 |
| syn_pcm (I_PCM, P_Skip, slice başına deblock ayarı) | 176x144 | 4 | bit-exact | bit-exact | 386 |
| syn_r_poc0 / poc1 (POC tip 0 / 1) | 176x144 | 10/10 | bit-exact | bit-exact | 200 / 200 |
| syn_r_ids (SPS id 7, PPS id 3 / 200) | 176x144 | 10 | bit-exact | bit-exact | 200 |
| syn_r_reorder (ref_pic_list_modification) | 176x144 | 10 | bit-exact | bit-exact | 200 |
| syn_r_mmco / mmco_reorder (MMCO 1/2/3/4/6) | 176x144 | 10/10 | bit-exact | bit-exact | 200 / 200 |
| syn_r_ltidr (uzun dönem IDR) | 176x144 | 10 | bit-exact | bit-exact | 200 |
| syn_r_ltidr_reorder_poc0_ids (birleşik) | 176x144 | 10 | bit-exact | bit-exact | 200 |
| syn_c_mmco_reorder | 352x288 | 10 | bit-exact | bit-exact | 195 |
| real_img_cif (fotoğraf: kahve fincanı, kaydırma + yakınlaştırma) | 352x288 | 30 | – | bit-exact | 200 |
| real_img_vga (fotoğraf: astronot) | 640x480 | 12 | – | bit-exact | 198 |

Son iki satır gerçek fotoğraflardan (scikit-image örnek görüntüleri) kamera hareketiyle üretilip
x264'ün olağan Baseline ayarlarıyla (preset medium, CRF 23) kodlanmış kliplerdir; RTL çıktısının
kaynak görüntüye göre PSNR'ı ~39 dB (CRF 23 kodlamanın kendisi), ffmpeg çıktısına göre ∞'dur.

34 akışın tamamı tb_top'ta bit-exact'tir; ayrıştırıcı izleri tb_parse'ta (hd1080 ve real_img_* hariç, orada çalıştırılmadı) bit-exact'tir. Ayrıştırıcı testbench'i ayrıca
iki benzetim iddiası (assertion) içerir: kod çözme aşamasında önceden hesaplanan tüketim miktarı ile
kullanım aşamasının tükettiği miktar her sembolde aynı olmalı, total_zeros / run_before tablo
numaraları bayat olmamalı; MC, iş parametreleri hazır olmadan okuma verisi gelirse durur.

### Çalıştırma

Gerekenler: GHDL ≥ 4 (`--std=08`), gcc, python3, test akışı üretmek için libx264'lü ffmpeg.

```sh
scripts/gen_streams.sh streams          # x264 akışları + ffmpeg referans YUV'ları (quick: yalnızca QCIF)
scripts/run_sim.sh model streams        # C modeli ↔ ffmpeg
scripts/run_sim.sh parse streams        # RTL ayrıştırıcı ↔ C modeli izi
scripts/run_sim.sh top   streams qcif_ip qcif_ref3   # tam RTL ↔ ffmpeg

# kendi fotoğrafınızdan kamera hareketli iki klip (real_img_cif, real_img_vga) üretip RTL'de çözmek:
IMG=foto.png scripts/gen_streams.sh streams quick
scripts/run_sim.sh top streams real_img_cif
```

`run_sim.sh` çıktı YUV'unu `sim/out/<ad>.rtl.yuv` olarak yazar; ör.
`ffmpeg -f rawvideo -pix_fmt yuv420p -s 352x288 -i sim/out/real_img_cif.rtl.yuv kare%02d.png`
ile karelere bakılabilir.

`scripts/gen_tables.py`, C modelindeki standart tablolarından `rtl/h264_tables_pkg.vhd`'yi üretir
ve VLC ROM'larını olası tüm 16 bitlik desenlerle doğrular.

## 6. Performans ve zamanlama

Ölçülen ortalama çevrim/MB değerleri yukarıdaki tablodadır. Tipik akışlarda **~208 çevrim/MB**
(darboğaz deblocking + çerçeve yazma ile ayrıştırıcı); çok yüksek bit hızlarında CAVLC'ye bağlı olarak
artar: ayrıştırıcı çevrim başına ~1.4–2 bit işler (qcif_noise: 1913 bit/MB → 1374 çevrim/MB).
I_PCM'de sınır 8 bit/çevrimlik giriş arayüzüdür (384 bayt/MB → ~380 çevrim/MB).

| Hedef | Gerekli MB/s | 208 çevrim/MB ile gereken saat |
|---|---|---|
| 1280x720 @ 30 | 108 000 | 22 MHz |
| 1920x1080 @ 30 | 244 800 | 51 MHz |
| 1920x1080 @ 60 | 489 600 | 102 MHz |

Hedef saat **100 MHz**'tir (`libero/h264_dec.sdc`): 1080p'de ~59 fps, 1080p30 için 2 kat pay.
Ayrıştırıcının CAVLC hızı 100 MHz'te ~140–200 Mbit/s'tir; Baseline seviye 4.x akışları (≤ 20–50 Mbit/s)
bunun çok altındadır.

Boru hattı (MB başına): mbparse (sembol başına 2 çevrim, VLC tüm türler paralel) → MC (iş parametresi
üreteci, satır başına bir AXI patlaması, 2 kademeli enterpolasyon) → recon (4x4 blok/çevrim, Intra4x4'te
blok bağımlılığı nedeniyle 3 çevrimde bir blok) → deblock (kenar başına 2 kademeli filtre, AW kanalı W
kanalının önünde).

### 6.1 Kaynak ve mantık derinliği tahmini (açık kaynak araçlar)

`scripts/synth_estimate.sh` ile (GHDL `--synth` + yosys `synth_microchip`, PolarFire hücreleri). Derinlik,
kayıttan kayda en uzun yolun LUT seviyesidir (taşıma zinciri bit başına 0.1 seviye); yönlendirme
gecikmesi ve modüller arası yollar dahil değildir. Kesin sonuçlar Libero (Synplify Pro + SmartTime)
raporlarından alınmalıdır.

| Modül | LUT (CFG1–4) | ARI1 | FF (SLE) | LSRAM | MACC | Derinlik (seviye) |
|---|---|---|---|---|---|---|
| `h264_nal` | 110 | 32 | 69 | 0 | 0 | 6.0 |
| `h264_bitreader` | 704 | 48 | 119 | 0 | 0 | 11.0 |
| `h264_hdr` | 4352 | 458 | 1960 | 3 | 0 | 18.0 |
| `h264_mbparse` | 9925 | 610 | 3965 | 6 | 5 | 19.5 |
| `h264_mc` | 14131 | 3902 | 6411 | 0 | 264 | 14.7 |
| `h264_recon` | 14828 | 3092 | 8720 | 7 | 176 | 16.0 |
| `h264_predbuf` | 6068 | 6 | 6272 | 0 | 0 | 7.0 |
| `h264_coefbuf` | 0 | 0 | 0 | 8 | 0 | 0.0 |
| `h264_deblock` | ~41.8K LUT4 (bkz. not) | – | 8930 | (7) | 0 | 14 (LUT4) |

Not: `h264_deblock` (büyük kayıt dizileri ve çoklayıcılar) yowasp-yosys'in 4 GB bellek sınırına
sığmadığı için yerel yosys 0.33 ile genel LUT4 eşlemesiyle sentezlendi (`synth_microchip` bu sürümde
yok); bu akışta toplayıcılar da LUT'a eşlendiğinden LUT sayısı PolarFire eşlemesine göre yüksek, derinlik
ise taşıma zincirleri LUT sayıldığı için kötümserdir. Satır tamponları (512x160 ve 512x96 bit) RAM olarak
çıkarıldı (~7 LSRAM). Kenar filtresi kademeleri ayrıca PolarFire eşlemesiyle ölçüldü (~10 seviye).
Yosys sabitle çarpmaları da MACC bloklarına eşler; Synplify bunların çoğunu mantıkla gerçekleştirir.
Toplam: deblock hariç ~50K LUT + 8K ARI1, ~28K FF, 24 LSRAM; deblock ile kabaca ~90K LUT, ~37K FF,
~31 LSRAM. MPFS250T'de 254K LE, 812 LSRAM ve 784 MACC vardır; çözücü cihazın kabaca %35–40'ını
kullanır.

## 7. MPFS250 Video Kit'e entegrasyon (Libero SoC)

Kitteki aygıt **MPFS250TS-1FCG1152I**'dır (-1 hız sınıfı). Kitte 4 GB DDR4 (x64) ve 2 GB LPDDR4 (x32)
bulunur; MSS DDR denetleyicisi en fazla 32 bit (+ECC) genişliği desteklediği için x64 DDR4'ün
fabric DDR denetleyicisine (**PF_DDR4**) bağlı olması beklenir. Bağlantıyı kitin kullanım kılavuzu /
şemasından doğrulayın. Çözücünün yalnızca FPGA kaynaklarıyla çalışması için çerçeve deposu bu
fabric DDR4'te tutulmalıdır (MSS ve RISC-V çekirdekleri kullanılmaz).

### 7.1 Tek başına sentez (kaynak/zamanlama kontrolü)

```sh
libero SCRIPT:libero/create_project.tcl SCRIPT_ARGS:"./h264_dec_prj"
```

Betik bir PolarFire SoC projesi oluşturur (`MPFS250TS`, `FCG1152`, `-1`, endüstriyel; kartınızdaki
aygıtla karşılaştırıp gerekirse dosyanın başındaki değişkenleri düzeltin), `rtl/*.vhd` dosyalarını VHDL-2008 olarak ekler,
`h264_dec_top`'u kök yapar, `libero/h264_dec.sdc`'yi (100 MHz) sentez, yerleştirme ve zamanlama
araçlarına bağlar ve sentezi çalıştırır. Zamanlamayı SmartTime ile kontrol edin; kapanmazsa SDC'deki
periyodu 13.333 ns'ye (75 MHz) çekin, 1080p30 yine karşılanır. Gerekli saat hızı bölüm 6'dadır.

### 7.2 SmartDesign'a ekleme

1. `rtl/*.vhd` dosyalarını `work` kütüphanesine ekleyin (Project Settings → VHDL: VHDL-2008),
   `h264_dec_top`'u SmartDesign'a sürükleyin.
2. Saat: PF_CCC'den tek saat (100 MHz; bkz. bölüm 6). Reset senkron ve aktif-yüksektir; CORERESET_PF
   çıkışını bu saat alanına senkronlayıp (aktif-düşükse tersleyerek) `rst`'ye bağlayın.
3. Bellek: `m_axi_*` (AXI4, 64 bit veri, 32 bit adres, ID 4 bit) → **CoreAXI4Interconnect** →
   PF_DDR4 AXI4 slave portu. PF_DDR4'ün kullanıcı AXI genişliği 64 bitten farklıysa interconnect
   genişlik dönüşümünü yapar; saat alanları farklıysa interconnect'te saat geçişini etkinleştirin.
   Ekran okuyucu ve bit akışı DMA'sı aynı interconnect'e ikinci/üçüncü master olarak bağlanabilir.
   `G_FB_BASE`, DDR penceresinde 72 MiB'lık boş, 4 MiB hizalı bir alanı göstermelidir.
4. Bit akışı kaynağı: `s_axis_*` (8 bit) arayüzüne herhangi bir fabric kaynağı bağlanabilir (DDR'dan
   okuyan basit bir DMA, Ethernet/UDP alıcısı, SD kart okuyucu...). Akışın son baytında `tlast=1`.
   Videoyu PC'den seri bağlantıyla göndermek için `h264_dec_top` yerine `h264_dec_uart_top` kullanın
   (bölüm 8; Libero betiğinde `SCRIPT_ARGS:"./h264_dec_prj h264_dec_uart_top"`), Ethernet üzerinden
   göndermek için `h264_dec_mss_top` (bölüm 9).
5. Ekran: `h264_dec_hdmi_top` bu işi yapar (`h264_disp`, bölüm 10): `o_frame_valid/index` ile gelen
   tamponu DDR'dan okur, RGB'ye çevirip HDMI'ya basar ve okuma bitince `i_release` ile geri verir.
   Kendi ekran okuyucunuzu bağlarsanız: tüketici tamponu tutarken çözücü o tamponu kullanmaz, en
   fazla 18 tampon döngüdedir; çıkış kare hızını ayarlamak tüketicinin görevidir.
6. `o_error`, `o_stream_done` ve `o_pic_count` bir LED'e / SmartDebug'a / Identify'a bağlanabilir.

### 7.3 Açık kaynak tahmini (Libero olmadan)

`scripts/synth_estimate.sh`, her modülü GHDL ile Verilog'a çevirip yosys `synth_microchip` ile
PolarFire hücrelerine eşler ve `scripts/pf_depth.py` ile en uzun kayıttan kayda yolu LUT seviyesi
olarak verir (taşıma zinciri bit başına 0.1 seviye sayılır). Sonuçlar bölüm 6.1'dedir; kesin
değerler Libero'dan alınmalıdır.

## 8. Bağlantı protokolü ve isteğe bağlı seri bağlantı

> Kiti PC'ye Ethernet ile bağlıyorsanız seri hat gerekmez, doğrudan bölüm 9'a geçin. Aşağıdaki
> protokol (8.2) Ethernet yolunda da aynen kullanılır.

```
 PC                                   MPFS250 Video Kit (yalnızca fabric)
 h264_send.py --serial PORT           ┌────────────────────────────────────────────────────┐
 ffmpeg ile Baseline'a dönüştürme ──► │ uart_rx ─► h264_uart_rx ─► h264_link ─► h264_dec_top│─► DDR4 / ekran
 (gerekirse), çerçeveleme, CRC  USB   │ uart_tx ◄─ h264_uart_tx ◄─ (32 KB FIFO)  AXI-Stream │
                         USB-UART ◄── └────────────────────────────────────────────────────┘
```

`h264_dec_uart_top` çözücünün AXI-Stream girişinin önüne bir UART ve bir protokol motoru koyar;
MSS / RISC-V kullanılmaz. PC tarafında Python programları videoyu gönderir.

### 8.1 Hangi seri port?

Kitteki CP2108 dört kanallı USB-UART köprüsünün MMUART_0/1 kanalları MSS'e (Linux konsolu)
bağlıdır; bunlar MSS olmadan kullanılamaz. Seçenekler:

* CP2108'in fabric I/O'ya giden bir kanalı varsa (kit şeması / kullanım kılavuzu DS00004698'den
  kontrol edin) onu kullanın; CP2108 en fazla 2 Mbaud'dur.
* Yoksa fabric'e giden bir başlığa 3.3 V'luk harici bir USB-UART dönüştürücü bağlayın
  (FT232H / FT2232H: 4 Mbaud 100 MHz'te tam bölünür; dönüştürücünün TXD'si → `uart_rx`,
  RXD'si ← `uart_tx`, ortak GND).

Pinler `libero/uart_pins.pdc` şablonunda yer tutucu olarak duruyor; kartınıza göre doldurun.
Generic'ler: `G_CLK_HZ` (100 MHz), `G_BAUD` (2 000 000), `G_FIFO_AW` (15 → 32 KB). `G_CLK_HZ / G_BAUD`
tam sayıya %2'den yakın olmalıdır. LED çıkışları: `led_rx` (çerçeve geliyor), `led_err` (CRC hatası
veya çözücü hatası), `led_done` (akış bitti).

| Baud | Yararlı veri hızı | Not |
|---|---|---|
| 2 000 000 | ~1.6 Mbit/s | CP2108 üst sınırı, 100 MHz / 50 |
| 4 000 000 | ~3.2 Mbit/s | FT232H/FT2232H, 100 MHz / 25 |

Bit hızı bağlantıdan yüksek bir akış da doğru çözülür, yalnızca gerçek zamandan yavaş oynar. Bu
yüzden dönüştürme yaparken programlar varsayılan bit hızını bağlantıya sığacak şekilde seçer
(2 Mbaud'da ~1.2 Mbit/s; 720p için yeterli kalite).

### 8.2 Protokol (özet; ayrıntılar `rtl/h264_link.vhd` başlığında)

* PC → FPGA çerçevesi: `55 AA | tür | sıra | uzunluk (≤ 1024) | veri | CRC16`. Türler: DATA, END (akış
  sonu, son bayta `tlast`), START (çözücüyü ve bağlantıyı sıfırla), STATUS (durum sor).
* FPGA → PC yanıtı (16 bayt): ACK / NAK / STATUS + FIFO'daki boş alan (kredi), çözülen resim sayısı,
  hata / bitti bayrakları, çözünürlük; CRC16 ile korunur.
* Akış kontrolü: PC en fazla 16 çerçeveyi yolda tutar ve FIFO'nun bildirdiği boş alandan fazlasını
  göndermez. CRC hatalı ya da sırası bozuk çerçevede FPGA NAK gönderir, PC beklenen çerçeveden
  itibaren yeniden gönderir (go-back-N); yanıt kaybolursa zaman aşımıyla yeniden gönderir. Tekrar
  gelen çerçeve yeniden onaylanır ama FIFO'ya ikinci kez yazılmaz.
* DATA verisi FIFO'ya geçici olarak yazılır, CRC doğruysa onaylanır; aksi hâlde geri alınır.

### 8.3 PC programları (`host/`)

Gerekenler: Python 3.8+, dönüştürme için PATH'te libx264'lü ffmpeg; seri hat için ayrıca
`pip install pyserial`. GUI (`h264_gui.py`) yalnızca Ethernet içindir (bölüm 9.4); seri hatta komut
satırı programı `--serial` ile kullanılır:

```sh
python host/h264_send.py --list                                    # seri portlar
python host/h264_send.py --serial COM5 video.264                   # Baseline .264 dosyasını olduğu gibi gönder
python host/h264_send.py --serial COM5 film.mp4 --size 1280x720 --bitrate 1.2M   # ffmpeg ile dönüştürerek
python host/h264_send.py --serial COM5 --status                    # çözücü durumu
```

Baseline olmayan dosyalar (`.mp4`, `.mkv`, High profil `.264`...) gönderilirken ffmpeg ile
Constrained Baseline Annex-B akışına dönüştürülür (geçici dosya olmadan, boru hattıyla). Program
her gönderimden önce START ile çözücüyü sıfırlar, sonunda END gönderir ve çözücü "akış bitti"
diyene kadar resim sayısını gösterir.

Kit olmadan denemek için `host/fpga_emu.py` FPGA tarafının yazılım modelidir (RTL ile aynı
protokol davranışı, sınırlı FIFO, yavaş çözücü ve hata enjeksiyonu); Linux/macOS'ta sözde
terminal üzerinde çalışır: `python3 host/fpga_emu.py --pty` → yazdığı `/dev/pts/N` ile
`h264_send.py --serial /dev/pts/N ...`.

### 8.4 Doğrulama

| Test | Kapsam | Sonuç |
|---|---|---|
| `scripts/run_sim.sh link streams <ad>` (`tb/tb_link.vhd`) | Python'un ürettiği çerçeveler UART üzerinden RTL'e; CRC hatalı çerçeve (NAK + yeniden gönderme), sıra dışı çerçeve, tekrar gönderilen çerçeve, küçük FIFO ile kredi bekleme, rastgele geri basınç, START; RTL yanıtları Python ayrıştırıcıyla denetlenir | qcif_ip, qcif_intra ve 300 KB rastgele veri (571 çerçeve, sıra numarası taşması): çıktı birebir, `tlast` son baytta |
| `scripts/run_sim.sh top-uart streams <ad>` | Akış UART + protokol üzerinden tam çözücüye (bir çerçeve bilerek bozuk) | qcif_ip, qcif_intra, real_coffee_cif (fotoğraf klibi, 30 kare): YUV ffmpeg ile bit-exact |
| `python3 host/test_link.py <akış.264>` | Gerçek pyserial ile PC programı ↔ FPGA modeli (sözde terminal): temiz hat, bayt hataları + kayıp yanıtlar, küçük FIFO + yavaş çözücü | qcif_ip, real_coffee_cif: tümü birebir |

Kaynak (yosys tahmini): `h264_link` ~500 LUT, 355 FF, 16 LSRAM (32 KB FIFO); UART'lar ~70 LUT.

## 9. PC'den kite video gönderme (Ethernet, MSS üzerinden)

```
 PC                                MPFS250 Video Kit
 h264_send.py / h264_gui.py        ┌─ MSS (tek çekirdek, Linux yok) ─┐     ┌─ fabric ─────────────────────────────────┐
 --ip 192.168.10.2       UDP 5000  │ GEM0 ─► kit/ yazılımı           │ APB │ h264_apb_bridge ─► h264_link ─► h264_dec_top │─► DDR4
 (bir datagram = bir çerçeve) ───► │ (ARP, ping, UDP; bayt taşır)    │────►│  (FIC3)            (32 KB FIFO) AXI-Stream │
                 RJ45 / VSC8662 ◄──│ SGMII                           │◄────│                                           │
                                   └─────────────────────────────────┘     └───────────────────────────────────────────┘
```

Kitteki Ethernet MAC'leri (GEM0/1) ve SGMII arayüzü MSS'in içindedir, fabric'ten doğrudan
kullanılamaz; onları bir işlemcinin sürmesi gerekir. Bunun için Linux gerekmez: tek bir çekirdekte
(varsayılan E51) çalışan küçük bir bare-metal program (`kit/`) yeterlidir. Program yalnızca bayt
taşır: gelen UDP datagramlarının içeriğini olduğu gibi fabric'e yazar, `h264_link`'in yanıtlarını
PC'ye geri yollar. Bölüm 8'deki protokol PC ile `h264_link` arasında **uçtan uca** aynen çalışır;
CRC denetimi, yeniden gönderme ve kredi tabanlı akış kontrolü fabric'te kalır, kaybolan bir
datagram yalnızca bir NAK / zaman aşımına mal olur. Çözme tamamen fabric'te yapılır.

### 9.1 Fabric tarafı (`h264_dec_mss_top`)

`h264_dec_mss_top` = `h264_apb_bridge` + `h264_link` + `h264_dec_top`. Libero betiğinde
`SCRIPT_ARGS:"./h264_dec_prj h264_dec_mss_top"`; SmartDesign'da APB3 slave portunu MSS'in FIC3 APB
initiator'una bağlayın (başka APB slave'ler de varsa CoreAPB3 üzerinden). Köprü adresin alt 12 bitini
çözer (4 KB pencere). FIC3 penceresi `0x4000_0000 – 0x5FFF_FFFF`'tir; yazılımdaki `FABRIC_BASE`
(varsayılan `0x40000000`) Libero bellek haritasındaki yuvanın adresine ayarlanmalıdır.

**Saat:** APB, çözücüyle **aynı saatte** (`clk`) olmalıdır; MSS Configurator'da FIC3 saatini
çözücünün PF_CCC çıkışından (100 MHz) besleyin. FIC3 başka bir saatte kalacaksa araya bir saat alanı
geçişi gerekir (dahil değil).

| Ofset | Yazmaç | Erişim | Anlamı |
|---|---|---|---|
| 0x00 | ID | R | `0x48323634` ("H264") |
| 0x04 | CAPS | R | [15:0] TX FIFO girdi sayısı (512), [31:16] RX FIFO bayt sayısı (64) |
| 0x08 | TXFREE | R | TX FIFO'daki boş girdi |
| 0x0C | TXDATA | W | 1 bayt ([7:0]) |
| 0x10 | TXDATA4 | W | 4 bayt ([7:0] ilk) |
| 0x14 | RXCOUNT | R | RX FIFO'daki bayt sayısı |
| 0x18 | RXDATA | R | [8] geçerli, [7:0] bayt; geçerliyse okuma baytı FIFO'dan alır |
| 0x1C | STATUS | R | [0] çözücü hatası, [1] akış bitti |
| 0x20 | PICTURES | R | çözülen resim sayısı |
| 0x24 | SIZE | R | [11:0] genişlik, [27:16] yükseklik |
| 0x28 | CTRL | W | [0]=1: bağlantıyı ve çözücüyü sıfırla, FIFO'ları boşalt |

Kaynak (yosys tahmini): köprü ~193 LUT, 163 FF, 1 LSRAM + 1 uSRAM, en uzun yol 7 LUT seviyesi
(+ `h264_link`, bölüm 8.4). LED'ler bölüm 8'deki gibidir.

### 9.2 MSS Configurator

* GEM0 (veya GEM1): SGMII, MDIO MSS I/O'larında. Video Kit'in referans MSS yapılandırmasında
  (HSS deposundaki `MSS_video-kit_mss_cfg.xml`) iki SGMII kanalı, MAC0/MAC1 saatleri ve MDIO pinleri
  (MSSIO 35/36) etkindir.
* FIC3 etkin (APB), saati çözücü saati (9.1).
* UART gerekmez: kit konsolu Ethernet üzerinden okunur (9.4). İsterseniz konsolun bir kopyası
  MMUART0'a da gider (115200, beklemesiz); tasarımınızda MMUART0 bağlı değilse yazılımı
  `KIT_UART_CONSOLE=0` ile derleyin.

### 9.3 Yazılım (`kit/`)

| Dosya | İçerik |
|---|---|
| `kit/src/net.c` | Ethernet / ARP / IPv4 / ICMP echo (ping) / UDP; parçalama yok, yanıt isteğin geldiği MAC'e gider |
| `kit/src/bridge.c` | UDP 5000 ↔ fabric: datagram baytları TXDATA4/TXDATA ile, yanıtlar 16 baytlık katlar hâlinde son göndericiye |
| `kit/src/fabric_mmio.c` | `h264_apb_bridge` yazmaç erişimi (`FABRIC_BASE`) |
| `kit/src/service.c` | UDP 5001: konsol metni ve durum bilgisi (sürüm, çalışma süresi, fabric bulundu mu, sayaçlar, çözücü) |
| `kit/src/log.c` | konsol: 8 KB'lık RAM halkası (PC buradan okur) + isteğe bağlı UART kopyası |
| `kit/src/kit_main.c` | ana döngü, bağlantı durumu, trafik varken 10 sn'de bir konsola istatistik |
| `kit/src/app_config.h` | MAC (`02:00:00:48:32:64`), IP (`192.168.10.2/24`), UDP portları (5000 video, 5001 konsol), `FABRIC_BASE` |
| `kit/mss/port_mss.c` | Microchip MSS MAC sürücüsü (GEM, VSC8662), SysTick, beklemesiz UART kopyası |
| `kit/mss/e51.c` | çekirdek giriş noktası (`KIT_HART=0` E51, `1` U54_1) |
| `kit/sim/` | aynı kodun Linux derlemesi: TAP arayüzü + `host/fpga_emu.py` (kit olmadan test) |

Derleme (SoftConsole): Microchip'in `polarfire-soc-bare-metal-examples` deposundaki
`driver-examples/mss/mss-ethernet-mac/mpfs-mac-simple-test` projesini temel alın (HAL, MSS MAC ve
VSC8662 PHY sürücüleri, bağlayıcı betikleri hazırdır):

1. Projedeki `src/application` altındaki uygulama dosyalarını çıkarın, yerine `kit/mss/e51.c`,
   `kit/mss/port_mss.c` ve `kit/src/*.c` ekleyin; `kit/src`'yi include yollarına ekleyin.
2. `fpga_design_config` klasörünü kendi Libero tasarımınızın MSS XML'inden üretilmiş olanla
   değiştirin (`platform/soc_config_generator/mpfs_configuration_generator.py`).
3. Tanımlar: örnek projedeki `TARGET_G5_SOC`, `MSS_MAC_SIMPLE_TX_QUEUE`; gerekirse `APP_IP`,
   `FABRIC_BASE` ve aşağıdaki kart ayarları (`-D...`).
4. E51'de LIM/DDR'dan hata ayıklayıcıyla çalıştırın ya da eNVM'e yazın; HSS ile açılıyorsa
   `KIT_HART=1` (U54_1). Konsol mesajları (IP, "Ethernet link up", fabric durumu, istatistikler)
   GUI'nin **Kit konsolu** sekmesinde ya da `h264_send.py --console` ile görünür.

**Kart ayarları — Video Kit şemasından doğrulayın.** Varsayılanlar Microchip'in Icicle Kit
örneğindeki VSC8662 bağlantısıdır; Video Kit için bunları donanımda doğrulayamadım:

| Tanım | Varsayılan | Anlamı |
|---|---|---|
| `KIT_GEM` | 0 | veri için kullanılan MAC |
| `KIT_MDIO_GEM` | 1 | PHY'nin MDIO hattının bağlı olduğu MAC (farklıysa o MAC yalnızca MDIO için başlatılır) |
| `KIT_PHY_ADDR` | `PHY_VSC8662_0_MDIO_ADDR` (8) | PHY'nin MDIO adresi; `MSS_MAC_AUTO_DETECT_PHY_ADDRESS` ile tarama |
| `kit_board_phy_reset()` | boş (weak) | PHY reset'i bir MSS GPIO'suna bağlıysa bu fonksiyonu yazıp bırakın |
| `FABRIC_BASE` | `0x40000000` | köprünün FIC3 adresi |

Yazılım önce ağı açar; fabric'teki köprüyü ilk istekte (bağlantı testi ya da gönderim) yoklar.
ID yazmacı yanlış okunursa konsola `no h264_apb_bridge at ...` yazar ve saniyede bir yeniden dener.
FIC3 hiç ayarlanmamışsa bu erişim işlemciyi kilitleyebilir; ağ önceden açıldığı için bağlantı testi
bu durumu "ping vardı, istekten sonra yanıt kesildi" diye ayırt eder.

**Ağ hiç çalışmıyorsa** (ör. PHY/MDIO ayarı yanlış, bağlantı ışığı yanmıyor) Ethernet konsolu da
çalışmaz; ilk kurulumda kiti yalnızca isteğe bağlı UART kopyası ya da SoftConsole hata ayıklayıcısı
gösterebilir.

### 9.4 PC tarafı

PC'nin Ethernet arayüzüne aynı alt ağdan sabit bir adres verin (ör. `192.168.10.1/24`), kiti doğrudan
ya da bir anahtar üzerinden bağlayın; `ping 192.168.10.2` yanıt vermelidir.

Kit DHCP kullanmaz; adresi yazılımda sabittir (`kit/src/app_config.h`: `APP_IP`, varsayılan
`192.168.10.2`) ve kit konsolunun ilk satırlarında yazar. Kiti mevcut bir ağa bağlayacaksanız o ağda boş bir
adresle derleyin (ör. `-DAPP_IP='IP4(192,168,1,50)'`).

**Arayüz (`python host/h264_gui.py`):** Python 3.8+ ve Tkinter yeterlidir (pyserial gerekmez);
dönüştürme için PATH'te libx264'lü ffmpeg. "Kit IP" ve "UDP port" alanlarını doldurun, önce
**Bağlantıyı test et**, sonra videoyu seçip **Gönder**. Test, yolu adım adım dener ve sorunun nerede
olduğunu söyler:

| Adım | Ne denetlenir | Başarısızsa |
|---|---|---|
| PC adresi | PC'nin kite hangi adresle gideceği (kitle aynı /24 ağda mı?) | PC'nin Ethernet kartına kitin ağından sabit adres verin |
| Ping | Kit ağda mı (yalnızca MSS yanıtlar) | kablo / bağlantı ışıkları, `APP_IP`, kitte yazılım çalışıyor mu |
| Kit yazılımı | UDP 5001 durum isteği: sürüm, çalışma süresi, Ethernet bağlantısı; **FPGA'daki köprü bulundu mu** | köprü bulunamadıysa: bitstream'de `h264_dec_mss_top`, `FABRIC_BASE`, FIC3; ping var ama yanıt yoksa: PC güvenlik duvarı ya da FIC3 erişiminde takılma (konsol daha önce yanıt verdiyse doğrudan takılma denir) |
| Durum sorgusu | MSS üzerinden fabric'teki `h264_link` yanıt veriyor mu; 10 sorguda gidiş-dönüş süresi | köprü var ama bu adım kalıyorsa: fabric saati / reset'i |

Test çözücüyü sıfırlamaz (yalnızca durum sorar); çözücü durumu (resim sayısı, çözünürlük, FIFO)
pencerenin alt satırında görünür.

**Kit konsolu sekmesi:** "kit konsolunu izle" işaretliyken GUI saniyede bir kitten yeni konsol
satırlarını ister (UDP port + 1 = 5001) ve gösterir; kit yeniden başlarsa "── kit yeniden başladı ──"
yazar. Kit son 8 KB'lık konsol metnini sakladığı için GUI'yi sonradan açsanız da açılış mesajlarını
görürsünüz.

**Komut satırı:**

```sh
python host/h264_send.py film.mp4 --size 1920x1080 --bitrate 8M    # kit 192.168.10.2 (varsayılan)
python host/h264_send.py --ip 192.168.1.50 video.264                # Baseline dosya olduğu gibi
python host/h264_send.py --status                                   # çözücü durumu
python host/h264_send.py --info                                     # kit yazılımı durumu (fabric bulundu mu, sayaçlar)
python host/h264_send.py --console                                  # kit konsolunu canlı izle (Ctrl-C)
ffmpeg -re -i giris.mp4 -an -vf format=yuv420p -c:v libx264 -profile:v baseline -tune zerolatency \
       -x264-params bframes=0:cabac=0:8x8dct=0:weightp=0 -b:v 6M -f h264 - \
  | python host/h264_send.py -                                      # canlı akış (stdin)
```

Kitin adresi `--ip` ile ya da `H264_KIT_IP` ortam değişkeniyle verilir; UDP portu `--port` (5000).
Ethernet'te dönüştürme için varsayılan bit hızı 8 Mbit/s'dir; yeniden gönderme zaman aşımı 30 ms'dir.
Windows'ta güvenlik duvarı Python'a gelen UDP yanıtlarını engellerse izin verin.

### 9.5 Doğrulama

| Test | Kapsam | Sonuç |
|---|---|---|
| `scripts/run_sim.sh apb streams <ad>` (`tb/tb_apb.vhd`) | Yazılımın yerine geçen APB master, Python'un ürettiği çerçeveleri TXDATA4/TXDATA ile yazar, yanıtları RXDATA'dan okur: TXFREE ile kredi bekleme, bozuk çerçeve (NAK + yeniden gönderme), tekrar çerçeve, END, CTRL sıfırlama; yanıtlar Python ayrıştırıcıyla denetlenir | qcif_ip, real_coffee_cif, hd720: çıktı birebir, `tlast` son baytta |
| `kit/sim/test_udp.sh <akış.264>` (Linux derlemesi, TAP + FPGA modeli; root) | Gerçek ağ yığını üzerinden ping ve `h264_send.py --ip ...`; `KIT_LOSS` ile iki yönde paket kaybı | 4.6 MB 1080p akış: kayıpsız 83 Mbit/s, %1 kayıp 26.7 Mbit/s, %5 kayıp 5.9 Mbit/s — hepsi birebir; ffmpeg'den canlı boru (1080p, 6 Mbit/s) birebir; GUI ile MP4 dönüştürerek gönderme |
| GUI, Xvfb altında `kit/sim`'e karşı | "Bağlantıyı test et": her şey çalışıyor; köprünün ID'si okunamıyor (`KIT_NO_FABRIC=1`); ilk FIC3 erişimi takılıyor (`KIT_FABRIC_HANG=1`); kit yok ve PC başka ağda; aynı ağda yanlış IP. Kit konsolu sekmesi; ardından gönderme (.264 dosyası; High profil MP4 dönüştürülerek, %1 paket kaybıyla) | Her durumda doğru adım başarısız ve doğru öneri gösterildi; konsol açılış mesajlarını ve fabric yoklamasını gösteriyor; gönderilen akışlar birebir, MP4'ten üretilen akış Constrained Baseline ve C modeliyle ffmpeg aynı çözüyor |
| RISC-V derleme denetimi | `kit/` yazılımının tamamı clang ile `rv64imac`/`lp64` hedefine, Microchip `platform` deposunun HAL ve MSS MAC/UART sürücü başlıklarına ve Video Kit referans MSS XML'inden üretilmiş `fpga_design_config`'e karşı derlendi (`-Wall -Wextra`, uyarısız); kullanılan tüm sürücü sembollerinin tanımlı olduğu kontrol edildi | Bağlanıp (link) donanımda çalıştırılmadı |

Kitte çalıştırmak için: 9.1–9.3'teki adres/saat/PHY ayarları doğrulanmalıdır. Simülasyonda hız
sınırı Python modelidir; kitte her 1 KB'lık çerçeve için bir kopya (alma kesmesi) ve 256 APB yazması
yapılır, bu da çözücünün ihtiyacının (1080p 8 Mbit/s ≈ 1 MB/s) çok üstünde olmalıdır (tahmin).

## 10. HDMI çıkışı (fabric, transceiver üzerinden)

```
 DDR4 (çerçeve deposu)                         fabric                                         kit
   │ AXI4 okuma (ARID 1)        ┌──────────────────────────────────────────────┐
   └──► h264_axi_rd_arb ◄──────►│ h264_disp_rd   kare kuyruğu, kare hızı,      │  4 satır yuvası
        (çözücü ARID 0 ile      │ (100 MHz)      satır okuma (Y, Cb, Cr)       │──► (iki saatli LSRAM)
         aynı okuma portu)      └──────────────────────────────────────────────┘        │
                                ┌──────────────────────────────────────────────┐        ▼
                                │ h264_disp_px  1080p60 zamanlama, YCbCr→RGB,  │  PF_XCVR (PMA, 4 hat × 40 bit,
                                │ (37.125 MHz,  renk çubukları, TMDS × 3       │──► 1485 Mbit/s) ──► PI3HDX1204
                                │  saatte 4 px) + TMDS saat hattı              │     re-driver ──► HDMI TX
                                └──────────────────────────────────────────────┘
```

`h264_dec_hdmi_top` = `h264_dec_mss_top` (Ethernet bağlantısı + çözücü) + `h264_axi_rd_arb` +
`h264_disp`. Kitte HDMI TX, SerDes 2 hatlarından bir re-driver (PI3HDX1204) üzerinden konnektöre
gider; referans saati 148.5 MHz'lik bir osilatördür (Video Kit kullanım kılavuzu). Bu yüzden görüntü
fabric'te TMDS sembollerine çevrilip transceiver'a (PF_XCVR, PMA modu) 40 bitlik sözcükler olarak
verilir; hat başına hız piksel saatinin 10 katıdır.

**DVI sinyali:** Sinyal DVI 1.0 TMDS kodlamasıdır (RGB, 8 bit, tam aralık). HDMI ekranlar DVI
sinyalini kabul eder; lisanslı HDMI TX çekirdeği gerekmez, ses ve InfoFrame yoktur. Hat düzeni
Microchip'in PolarFire HDMI 2.0 referans bileşeniyle aynıdır: LANE0 = TMDS saati (`0000011111`),
LANE1 = mavi (HSYNC/VSYNC), LANE2 = yeşil, LANE3 = kırmızı; piksel tarafı LANE0'ın TX saatiyle
(37.125 MHz = 148.5 MHz / 4) çalışır.

**Davranış**
* Resim ekranın ortasına konur (yatay başlangıç 16'nın katına yuvarlanır); ekrandan büyükse
  ortadan kırpılır. Ölçekleme yoktur (1080p ekranda 720p video ortada, çerçeveli görünür).
* Renk dönüşümü: 720 satır ve üstü BT.709, altı BT.601 (sınırlı aralık YCbCr → tam aralık RGB,
  10 bit sabit noktalı katsayılar); kroma en yakın örnekle çoğaltılır.
* İlk kare gelene kadar, ekran kapalıyken ya da istenince %75 renk çubukları gösterilir. HDMI
  bağlantısını videodan bağımsız denemek için bunu kullanın (GUI: "renk çubukları",
  `h264_send.py --bars`).
* Kare hızı: Ekran 60 Hz'dir. Her ekran karesinde videonun kare hızına göre (ör. 25 fps'te 60
  karede 25 kez) bir sonraki kareye geçilir. Kare hızını GUI ve `h264_send.py` dosyadan bulup
  (ham akışta SPS VUI, kapsayıcıda ffprobe; yoksa 30) gönderimden önce kite yazar.
* Ekran kareleri tuttuğu için çözücü oynatma hızında çalışır. FIFO dolunca PC'nin gönderimi de
  kredi beklemesiyle yavaşlar; dosya gerçek zamanlı oynatılır. HDMI saati yoksa (transceiver
  hazır değil) kareler hemen geri verilir, çözme durmaz.
* "Akış bitti" bayrağı ekran bekleyen son kareyi de aldıktan sonra kalkar. PC yeni bir video
  başlattığında (START → çözücü sıfırlanır) ekran eski kareleri geri vermeden unutur; o ekran
  karesinin kalanı renk çubuğu olur, yeni video ilk karesiyle başlar.
* Ekran okuyucunun yükü: 1080p60'ta satır başına 3840 bayt, yaklaşık 280 MB/s; DDR4 için
  sorun değildir.

**Yazmaçlar** (APB penceresi, bölüm 9.1; kit yazılımı `H264R` komutuyla yazar):

| Ofset | Yazmaç | Anlamı |
|---|---|---|
| 0x2C | DISP_CTRL | [0] ekran kapalı (kareler hemen geri verilir), [1] renk çubukları, [2] TMDS sözcüklerinin bit sırası ters, [31:16] videonun kare hızı × 100 (varsayılan 3000) |
| 0x30 | DISP_STAT | [15:0] kare geçişi sayısı, [31:16] verisi zamanında gelmediği için siyah gösterilen satır sayısı |

**Kaynak (yosys tahmini):**
* Okuyucu (`h264_disp_rd`, 100 MHz): ~1600 LUT, ~1650 FF, en uzun yol 11.7 seviye; çözücü
  modüllerinin altında.
* Piksel tarafı (`h264_disp_px`, 37.125 MHz, üç TMDS kodlayıcı dahil): ~2550 LUT, ~1250 FF,
  56 MACC. En uzun yol 21.5 seviye; bu, dört sembol boyunca zincirlenen TMDS DC dengesidir ve
  26.9 ns'lik saat periyoduna sığar.
* Hakem: ~50 LUT.
* Satır yuvaları: Y 1024×64 + Cb/Cr 2×512×64 bit (~8 LSRAM).

### 10.1 Libero entegrasyonu

1. Projeye `rtl/*.vhd`'yi ekleyin (`create_project.tcl` ile kök `h264_dec_hdmi_top`;
   `libero/h264_hdmi.sdc` otomatik eklenir).
2. `source libero/hdmi_phy.tcl` ile transceiver bileşenlerini ve `HDMI_PHY` SmartDesign'ını
   oluşturun: PF_XCVR_REF_CLK (diferansiyel, 148.5 MHz), PF_TX_PLL, PF_XCVR_ERM (PMA modu, 4 hat,
   TX arayüzü 40 bit, 1485 Mbit/s). **Bu betik Libero'da çalıştırılmadı.** Parametre adları
   Microchip'in HDMI 2.0 örneğinden alındı. Çalıştırdıktan sonra PF_XCVR yapılandırıcısında TX hızının
   1485 Mbit/s, TX saatinin 37.125 MHz olduğunu ve TX PLL'in istenen frekansta olduğunu kontrol edin.
3. SmartDesign bağlantıları:

   | h264_dec_hdmi_top | HDMI_PHY |
   |---|---|
   | `o_tmds_clk` / `o_tmds_b` / `o_tmds_g` / `o_tmds_r` | `LANE0_TX_DATA` / `LANE1_TX_DATA` / `LANE2_TX_DATA` / `LANE3_TX_DATA` |
   | `hdmi_clk`, `hdmi_clk_stable` | `TX_CLK`, `TX_CLK_STABLE` |
   | `m_axi_*` | PF_DDR4 AXI portu (genişlik farklıysa CoreAXI4Interconnect) |
   | `paddr` … `pslverr` | MSS FIC3 APB (bölüm 9.1) |

4. I/O Editor'da PF_XCVR'ı HDMI TX re-driver'ına giden SerDes 2 hatlarına, referans saatini
   148.5 MHz osilatörün pinlerine yerleştirin (kılavuzda AF29/AF30). **Kitin şemasından
   doğrulayın:** hangi hattın TMDS saati/D0/D1/D2 olduğu (farklıysa SmartDesign'da 40 bitlik
   veri yollarının bağlantısını değiştirin), P/N yönü ve re-driver'ın etkinleştirme pini.
5. Başka bir mod için generic'leri değiştirin. Yatay toplam 4'ün katı olmalıdır. 720p60'ın CEA
   toplamı 1650'dir ve 4'e bölünmez; `G_H_FP => 108` ile 1648 kullanın (60.07 Hz, ekranlar kabul eder).
   Hat hızı 742.5 Mbit/s'dir.

**Çalıştırma sırası önerisi:**
1. Bitstream'i yükleyin; video göndermeden ekranda renk çubukları görünmelidir.
2. Görüntü yoksa transceiver'ın bit sırasını deneyin: `h264_send.py --bars --bitrev` ya da GUI'de
   "TMDS bit sırası ters".
3. Sonra videoyu gönderin. `h264_send.py --info` çıktısındaki `disp_frames` ve `disp_late_lines`
   değerleri ekranın durumunu gösterir.

### 10.2 Doğrulama

`scripts/run_sim.sh disp <akış_dizini> <ad>` (`tb/tb_disp.vhd`) şu zinciri çalıştırır: akış →
çözücü → DDR modeli (rastgele geri basınç, okuma gecikmesi) → hakem → `h264_disp` → TMDS.
Testbench TMDS sembollerini geri çözer:
* Saat hattı deseni, her sembolün geçerliliği ve DC dengesi denetlenir.
* Satır/kare uzunlukları ve sync genişlikleri denetlenir.
* Tam kareler dosyaya yazılır; `scripts/disp_check.py` bunları ffmpeg referansından aynı kurallarla
  üretilen beklenen RGB görüntüyle bit bit karşılaştırır.

Simülasyon küçük bir ekran zamanlamasıyla yapılır (varsayılan 256x192, `DISP=WxH`).

| Test | Sonuç |
|---|---|
| qcif_ip (176x144, ortada) | 2 renk çubuğu karesinden sonra 8 karenin her biri sırayla bir kez gösterildi, RGB birebir; 0 TMDS hatası |
| qcif_ip, ekran 160x128 (ortadan kırpma) | 8 kare birebir |
| odd_100x60 (kırpmalı akış) | 6 kare birebir |
| qcif_ip, 30 fps (`RATE=3000`) | her kare iki ekran karesi boyunca gösterildi |
| hd720 (BT.709, ekran 256x192'ye ortadan kırpılmış) | 3 kare BT.709 dönüşümüyle birebir, 0 TMDS hatası (simülasyonda 720p çözümü ekrandan yavaş olduğu için kareler tekrarlandı; 0. ve 1. kare kırpılan bölgede aynıdır) |
| qcif_ip, `RESTART=1`: akış bitince çözücü sıfırlanır, akış yeniden gönderilir (PC'den yeni video) | ilk turda 8 kare, kısa renk çubuğu geçişi, ikinci turda yine 8 kare birebir; çözücü hatası yok |
| Kit yazılımı / PC (`kit/sim`) | `--bars`, VUI'den okunan 25 fps `DISP_CTRL`'e yazıldı |

Her çalıştırmada sıfırlamadan sonraki ilk satır siyah sayılır (`late lines = 1`); okuyucunun ilk
satırı hazırlamasından önce gelir ve beklenen bir durumdur.

## 11. Dizin yapısı

```
model/    C referans modeli (h264_model.c)
rtl/      VHDL-2008 RTL (çözücü; h264_uart / h264_link / h264_dec_uart_top: seri bağlantı;
          h264_apb_bridge / h264_dec_mss_top: MSS üzerinden Ethernet; h264_disp / h264_tmds /
          h264_axi_rd_arb / h264_dec_hdmi_top: HDMI çıkışı)
tb/       GHDL testbench'leri (tb_parse, tb_top, tb_link, tb_apb, tb_disp, tb_util_pkg)
scripts/  gen_streams.sh, stream_tools.py, gen_tables.py, run_sim.sh, yuvcmp.py,
          synth_estimate.sh, pf_depth.py, disp_check.py
host/     PC programları: h264_send.py (komut satırı), h264_gui.py (arayüz), h264link.py (protokol,
          seri ve UDP), fpga_emu.py (FPGA tarafının yazılım modeli), test_link.py, make_vectors.py
kit/      MSS bare-metal Ethernet köprüsü: src/ (taşınabilir), mss/ (Microchip HAL portu),
          sim/ (Linux/TAP derlemesi)
libero/   create_project.tcl (Libero SoC projesi + sentez), h264_dec.sdc, h264_hdmi.sdc,
          hdmi_phy.tcl (HDMI transceiver bileşenleri), uart_pins.pdc (şablon)
```

## 12. Bilinen sınırlar / yol haritası

* Main/High profil (CABAC, B slice, 8x8 dönüşüm) desteklenmiyor.
* Çıkış sırası çözme sırasıdır; B slice olmayan akışlarda bu, gösterim sırasıyla aynıdır.
* Hata gizleme (error concealment) yok; eksik slice'lı bir resim tamamlanmaz.
* Zamanlama yalnızca açık kaynak araçlarla tahmin edildi (bölüm 6.1); Libero'da doğrulanmalıdır.
* Seri PC bağlantısı UART hızıyla sınırlıdır (2–4 Mbaud). Ethernet yolu (bölüm 9) simülasyonda ve
  RISC-V derleme denetiminde doğrulandı, kitte çalıştırılmadı; Video Kit'in PHY/MDIO bağlantısı,
  PHY reset'i ve FIC3 adresi/saati kart üzerinde doğrulanmalıdır. FIC3 APB'si çözücüyle aynı saatte
  olmalıdır (saat alanı geçişi yok). Kit konsolu Ethernet üzerindendir; ağ hiç kurulamıyorsa ilk
  kurulumda UART kopyası ya da hata ayıklayıcı gerekebilir.
* HDMI çıkışı simülasyonda doğrulandı, kitte çalıştırılmadı. Transceiver ayarları (`hdmi_phy.tcl`),
  hat sırası/yönü ve re-driver ayarı kart üzerinde doğrulanmalıdır. Çıkış DVI sinyalidir (ses ve
  InfoFrame yok); ölçekleme yoktur, kroma en yakın örnekle çoğaltılır.
* Olası iyileştirmeler: deblock'ta MB yüklemesini önceki MB'nin çıkışıyla örtüştürmek, CAVLC'de
  level + run_before'u tek adımda çözmek, tahmin tamponunu LSRAM'e taşımak.
