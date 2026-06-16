# Vision / EKF Debug Plan

Bu klasor ana SIFT sisteminden bagimsiz debug icindir. Amac sirayla sunlari ayirmak:

1. MAVLink baglantisi dogru mu?
2. ArduPilot parametreleri dogru mu?
3. SIFT olmadan, ciplak `VISION_POSITION_ESTIMATE` EKF'e ulasiyor mu?
4. GPS kapatinca sorun MAVLink/EKF tarafinda mi, yoksa SIFT/NED tarafinda mi?

Ana kural: `live_sift_nav.py` calismasin. Bu testleri tek tek calistir. TCP kullaniyorsan stream icin `5762`, izleme icin `5763` kullanmak daha temiz oluyor.

## Aktif calisma hedefi - SIFT estimator/filter

VPE-only XY tarafi artik temel olarak dogrulandi. Sonuc:

```text
VPE-only XY source set 2 calisiyor.
VSE aktif hedef icin gerekmiyor.
VPE z=0 basilebilir; altitude barodan geliyor.
VPE roll/pitch/yaw=0 basilebilir; yaw compass/manyetometreden geliyor.
Kalan problem raw SIFT fix jitter/outlier davranisi.
```

Su anki ana hedef:

```text
raw SIFT fix -> visual temporal estimator/filter -> VPE publisher
```

Ek olarak yeni master-map patch DB hazirlandi. Bu yol eski tile DB'yi silmez;
runtime'da `--map-source sift-master` ile secilir.

```text
QGIS/SAU CAMPUS/output/SIFT/patches/
patch grid = 11x11 = 121
patch size = 2048 px, step = 1024 px
runtime normal ROI = 3x3 patch, yani --max-tiles-per-scale 9
```

Detayli karar, baseline komut ve sonraki implementasyon plani:

```text
vision_debug/SIFT_NEXT_STEPS.md
```

Test detaylari ve sayisal log:

```text
vision_debug/TEST_LOG.md
```

## Onceki hedef - VPE-only XY

Su an birinci hedefimiz SIFT'i karistirmadan sunu kanitlamak:

```text
ArduPilot navigasyonu sadece ExternalNav XY pozisyonu ile yapabiliyor mu?
```

Bu testte sadece `VISION_POSITION_ESTIMATE` basilir. `VISION_SPEED_ESTIMATE`
basilmaz ve EKF source set 2 velocity fusion kapali tutulur:

```text
EK3_SRC2_POSXY=6   # ExternalNav XY
EK3_SRC2_VELXY=0   # VSE kullanma
EK3_SRC2_POSZ=1    # baro
EK3_SRC2_VELZ=0
EK3_SRC2_YAW=1     # compass
```

`VISION_POSITION_ESTIMATE` mesaji z/roll/pitch/yaw alanlarini teknik olarak
tasir, ama yukaridaki source ayarlarinda amacimiz EKF'e sadece XY katkisi
vermektir. Z barodan, yaw compass/manyetometreden, hiz/ivme de arac uzerindeki
sensorlerden gelecek.

Bu nedenle ilk test sirasi:

1. Source set 1 = GPS, source set 2 = VPE-only ExternalNav parametrelerini set et.
2. GPS ile kalk ve 80 m'ye cik.
3. Gazebo ground truth bridge'i `--no-send-speed` ile sadece VPE basacak sekilde calistir.
4. Korumali olarak source set 2'ye gec.
5. Hover ve kucuk Go To hareketlerinde EKF variance, LAND, yaw sapmasi ve pozisyon kontrolunu izle.

Bu test temiz olmadan SIFT optimizasyonuna gecme. Temiz olursa siradaki ana is
SIFT'i EKF'den bagimsiz benchmark ederek 4-5 Hz taze ve guvenilir fix'e cikarmak.

## Test 1 - MAVLink durumunu oku

SITL ve MAVProxy calisirken:

```bash
python3 vision_debug/mavlink_doctor.py \
  --mavlink udpin:127.0.0.1:14550 \
  status \
  --duration 6
```

Bana su bloklari aynen at:

- `HEARTBEAT`
- `PARAM`
- `LOCAL_POSITION_NED`
- `ATTITUDE`
- `EKF_STATUS_REPORT`
- `STATUSTEXT`

Beklenen: heartbeat gelmeli, local position ve attitude akmali.

## Test 2 - ExternalNav parametrelerini set et

GPS kapatmadan once:

```bash
python3 vision_debug/mavlink_doctor.py \
  --mavlink udpin:127.0.0.1:14550 \
  set-params
```

Sonra SITL'i reboot/restart et. `VISO_TYPE` reboot ister.

Restart sonrasi Test 1'i tekrar calistir. Parametrelerde en az sunlari gormeliyiz:

```text
VISO_TYPE=1
VISO_POS_M_NSE=2.0
VISO_VEL_M_NSE=2.0
EK3_SRC1_POSXY=6
EK3_SRC1_POSZ=1
EK3_SRC1_YAW=1
```

## Test 3 - SIFT olmadan VISION_POSITION_ESTIMATE yolla

Bu test goruntu kullanmaz. ExternalNav primary olduktan sonra `LOCAL_POSITION_NED` henuz gelmeyebilir; o durumda ilk pose'u manuel `x=0 y=0 z=0` seed olarak yolla. EKF `POSZ=1` oldugu icin z barodan gelecek, bu testte asil baktigimiz x/y akisi.

Ilk enjeksiyon:

```bash
python3 vision_debug/mavlink_doctor.py \
  --mavlink udpin:127.0.0.1:14550 \
  stream-vision \
  --pose-source manual \
  --x 0 \
  --y 0 \
  --z 0 \
  --rate 10 \
  --duration 30
```

Beklenen:

- Script `sent=` sayisini arttirmali.
- Bir sure sonra `local=(...)` gorunmeye baslamali.
- ArduPilot `EKF variance` yazmamali.
- `STATUSTEXT` icinde `stopped aiding` gelmemeli.

MAVProxy console'da `EKF3 IMU... is using external nav data` gorursen MAVLink/VPE yolu calisiyor demektir. Script bittikten sonra `PreArm: VisOdom: not healthy`, `EKF variance` veya `stopped aiding` gelmesi normaldir; ExternalNav primary iken vision stream surekli akmak zorunda.

Bu test bile fail olursa sorun SIFT degil; MAVLink route, `VISO_TYPE`, EKF source veya paket formatidir.

Bu temizse ikinci kez local takip testi yap:

```bash
python3 vision_debug/mavlink_doctor.py \
  --mavlink udpin:127.0.0.1:14550 \
  stream-vision \
  --pose-source local \
  --bootstrap-sec 5 \
  --x 0 \
  --y 0 \
  --z 0 \
  --rate 10 \
  --duration 30
```

Bu test `LOCAL_POSITION_NED` gelene kadar manuel `0,0,0` yollar. Local stream gelirse otomatik `pose[local]` moduna gecer.

## Test 4 - GPS kapatma testi

Bir terminalde uzun vision stream baslat:

```bash
python3 vision_debug/mavlink_doctor.py \
  --mavlink udpin:127.0.0.1:14550 \
  stream-vision \
  --pose-source manual \
  --x 0 \
  --y 0 \
  --z 0 \
  --rate 10 \
  --duration 0
```

Baska terminalde MAVProxy console'da:

```text
param set SIM_GPS_DISABLE 1
```

Beklenen: `EKF variance`, `EKF3 IMU stopped aiding`, otomatik `LAND` gelmemeli.

Eger Test 3 iyi ama Test 4 fail ise EKF external nav'i GPS varken tolere ediyor ama GPS gidince source/fusion tarafinda eksik var demektir.

## Test 4b - Velocity source ihtimalini ayir

`EKF3 is using external nav data` gorulmesine ragmen `LOCAL_POSITION_NED` hic gelmiyorsa EKF yatay pozisyonu saglikli flag'lemiyor olabilir. Once velocity source'u ExternalNav yap:

```bash
python3 vision_debug/mavlink_doctor.py \
  --mavlink udpin:127.0.0.1:14550 \
  set-params \
  --velxy-external
```

Sonra SITL'i restart et. Ardindan hem pose hem sifir hiz bas:

```bash
python3 vision_debug/mavlink_doctor.py \
  --mavlink udpin:127.0.0.1:14550 \
  stream-vision \
  --pose-source manual \
  --x 0 \
  --y 0 \
  --z 0 \
  --vx 0 \
  --vy 0 \
  --vz 0 \
  --send-speed \
  --rate 10 \
  --duration 60
```

Beklenen: `ekf_flags` icinde `POS_HORIZ_REL` veya `POS_HORIZ_ABS` gorunmesi. Bunlar gelirse live sistemde sadece VPE degil, velocity paketi de basmamiz gerekecek.

## Test 4c - Ikinci porttan EKF/local izle

Stream calisirken ikinci bir MAVLink cikisi ac:

```text
output add 127.0.0.1:14551
```

Terminal 1'de stream'i acik birak:

```bash
python3 vision_debug/mavlink_doctor.py \
  --mavlink udpin:127.0.0.1:14550 \
  stream-vision \
  --pose-source manual \
  --x 0 \
  --y 0 \
  --z 0 \
  --vx 0 \
  --vy 0 \
  --vz 0 \
  --send-speed \
  --rate 10 \
  --duration 0
```

Terminal 2'de ikinci porttan sadece durum oku:

```bash
python3 vision_debug/mavlink_doctor.py \
  --mavlink udpin:127.0.0.1:14551 \
  status \
  --duration 10
```

Eger 14551'de de `LOCAL_POSITION_NED <not received>` ise EKF horizontal position valid etmiyor. Eger 14551'de local geliyorsa sorun bizim 14550 channel/request tarafindadir.

## Test 4d - VPE-only ve VPE+VSE A/B testi

Bu testin amaci `VISION_POSITION_ESTIMATE` tek basina yeterli mi, yoksa `VISION_SPEED_ESTIMATE` de gerekir mi sorusunu ayirmak.

### A: Sadece VISION_POSITION_ESTIMATE

Source1'i ExternalNav position-only yap:

```bash
python3 vision_debug/mavlink_doctor.py \
  --mavlink tcp:127.0.0.1:5762 \
  set-params
```

SITL'i restart et. Terminal 1'de sadece pose bas:

```bash
python3 vision_debug/mavlink_doctor.py \
  --mavlink tcp:127.0.0.1:5762 \
  stream-vision \
  --pose-source manual \
  --x 0 \
  --y 0 \
  --z 0 \
  --rate 10 \
  --duration 0
```

Terminal 2'de izle:

```bash
python3 vision_debug/mavlink_doctor.py \
  --mavlink tcp:127.0.0.1:5763 \
  status \
  --duration 10
```

Karsilastirma icin kaydet: `LOCAL_POSITION_NED` var mi, `EKF_STATUS_REPORT flags` icinde `POS_HORIZ_REL`, `POS_HORIZ_ABS`, `PRED_POS_HORIZ_REL`, `PRED_POS_HORIZ_ABS` var mi?

### B: VISION_POSITION_ESTIMATE + VISION_SPEED_ESTIMATE

Source1'i ExternalNav position + velocity yap:

```bash
python3 vision_debug/mavlink_doctor.py \
  --mavlink tcp:127.0.0.1:5762 \
  set-params \
  --velxy-external
```

SITL'i restart et. Terminal 1'de pose ve sifir hiz bas:

```bash
python3 vision_debug/mavlink_doctor.py \
  --mavlink tcp:127.0.0.1:5762 \
  stream-vision \
  --pose-source manual \
  --x 0 \
  --y 0 \
  --z 0 \
  --vx 0 \
  --vy 0 \
  --vz 0 \
  --send-speed \
  --rate 10 \
  --duration 0
```

Terminal 2'de tekrar izle:

```bash
python3 vision_debug/mavlink_doctor.py \
  --mavlink tcp:127.0.0.1:5763 \
  status \
  --duration 10
```

Beklenen: B testinde `LOCAL_POSITION_NED` gelmeli ve flags degeri 831 civari olup horizontal position flag'leri aktif olmali. Bu olursa pratik cevap sudur: SIFT sistemi VPE ile beraber VSE de basacak.

## Test 5 - 80 m kalkis icin source-set akisi

SIFT'in kalkis sirasinda guvenilir olmasini beklemiyoruz. Kamera irtifasi, olcek ve gorunen alan surekli degistigi icin en temiz akista GPS ile kalkilir, 80 m civarinda SIFT saglam fix verince ExternalNav'a gecilir.

Bir kez kaynak setlerini hazirla:

```bash
python3 vision_debug/mavlink_doctor.py \
  --mavlink tcp:127.0.0.1:5762 \
  set-takeoff-switch-params
```

SITL'i restart et. Source set 1 GPS, source set 2 VPE-only ExternalNav olacak:

```text
SIM_GPS_DISABLE=0
VISO_VEL_M_NSE=2.0
EK3_SRC1_POSXY=3
EK3_SRC1_VELXY=3
EK3_SRC2_POSXY=6
EK3_SRC2_VELXY=0
EK3_SRC2_POSZ=1
EK3_SRC2_YAW=1
```

`VISO_TYPE=1` global bir pre-arm check acar. Bu yuzden source set 1 GPS olsa bile arm etmeden once VisualOdom'a paket gelmeli. Kalkis oncesinde dummy stream'i acik tut:

```bash
python3 vision_debug/mavlink_doctor.py \
  --mavlink tcp:127.0.0.1:5762 \
  stream-vision \
  --pose-source manual \
  --x 0 \
  --y 0 \
  --z 0 \
  --rate 10 \
  --duration 0
```

Bu stream GPS source set aktifken navigasyon icin degil, sadece `VisOdom: not healthy` pre-arm check'ini gecmek icindir. 80 m'de gercek SIFT fix'i saglamlasmadan source set 2'ye gecme.

VPE+VSE karsilastirma testi yapmak istersen ayni param komutunu su flag ile calistir:

```bash
python3 vision_debug/mavlink_doctor.py \
  --mavlink tcp:127.0.0.1:5762 \
  set-takeoff-switch-params \
  --src2-velxy-external
```

Bu flag `EK3_SRC2_VELXY=6` yapar. VPE-only calismada kullanma.

Kalkis:

1. Source set 1'de kal, GPS acik olsun.
2. Dummy vision stream acikken `pre-arm good` bekle.
3. Araci 80 m'ye kaldir.
4. Aktif VPE-only calisma icin Gazebo ground truth bridge'i baslat.
5. Source set 2'ye once korumali komutla gec:

```bash
python3 vision_debug/mavlink_doctor.py \
  --mavlink udp:127.0.0.1:14550 \
  safe-switch-source-set \
  --to-set 2 \
  --rollback-set 1
```

Bu komut once EKF sagligini kontrol eder, sonra source set 2'ye gecer. `velocity_variance`, `pos_horiz_variance` veya mode `LAND` kotuye giderse otomatik source set 1'e doner. `ROLLBACK set=1` gorursen ExternalNav henuz ucus icin saglam degildir.

Manuel gecis sadece sistemin stabil oldugundan eminsen kullan:

```bash
python3 vision_debug/mavlink_doctor.py \
  --mavlink udp:127.0.0.1:14550 \
  switch-source-set \
  --set 2
```

Geri GPS'e donmek icin:

```bash
python3 vision_debug/mavlink_doctor.py \
  --mavlink udp:127.0.0.1:14550 \
  switch-source-set \
  --set 1
```

`SIM_GPS_DISABLE=1` testini sadece source set 2 saglikli calisirken dene.

## Test 6 - Sonra SIFT/NED tarafina don

Test 1-5 temiz olmadan SIFT'e donmeyelim. Temiz olursa sonraki adim:

- SIFT'in verdigi NED ile `LOCAL_POSITION_NED` arasindaki farki olcecegiz.
- Drone sabitken SIFT sonucu ziplama yapiyor mu bakacagiz.
- Sonra ROI ve hiz tahmini tarafini ekleyecegiz.

## Test 7 - Gazebo ground truth ile VPE-only ExternalNav testi

Bu test SIFT'i tamamen aradan cikarir. Gazebo model pose'u okunur, ilk anda ArduPilot `LOCAL_POSITION_NED` ile hizalanir, sonra gercek Gazebo deltasi `VISION_POSITION_ESTIMATE` olarak basilir.

Amac:

- Bu test stabilse sorun SIFT/NED/harita eslesmesindedir.
- Bu test de failsafe yaparsa sorun EKF source, VPE timing veya parametre tarafindadir.

80 m'ye GPS ile ciktiktan sonra live SIFT'i kapat ve ground truth bridge'i sadece VPE basacak sekilde ac:

```bash
python3 vision_debug/gazebo_truth_bridge.py \
  --mavlink udp:127.0.0.1:14550 \
  --gz-topic /world/iris_runway/pose/info \
  --model iris_with_gimbal \
  --axis enu \
  --rate 4 \
  --no-send-speed \
  --speed-source zero \
  --duration 0
```

Bridge `BOOTSTRAP` ve `GT sent=...` satirlari basmali. `err=(...)` GPS local NED ile ground truth ExternalNav arasindaki farktir. Hover'da bu fark kucuk ve sakin kalmali.

Sonra ayri terminalde korumali gecis yap:

```bash
python3 vision_debug/mavlink_doctor.py \
  --mavlink udp:127.0.0.1:14550 \
  safe-switch-source-set \
  --to-set 2 \
  --rollback-set 1 \
  --monitor-sec 0
```

Bu testte `GT sent=... vel=(0.00,0.00,0.00)` gorunebilir; bu sadece bridge'in speed kaynagini sifir tuttugunu gosterir. `--no-send-speed` oldugu icin EKF'e VSE paketi gitmez.

RPY alanlarinin etkisini izole etmek icin ayni testi zero attitude ile tekrar edebilirsin:

```bash
python3 vision_debug/gazebo_truth_bridge.py \
  --mavlink udp:127.0.0.1:14550 \
  --gz-topic /world/iris_runway/pose/info \
  --model iris_with_gimbal \
  --axis enu \
  --rate 4 \
  --no-send-speed \
  --speed-source zero \
  --attitude-source zero \
  --duration 0
```

Beklenti: `EK3_SRC2_YAW=1` oldugu icin yaw compass'tan gelmeli ve zero RPY
VPE-only source set 2 davranisini degistirmemeli. Degistirirse ArduPilot VPE
attitude alanlarina bekledigimizden daha hassas demektir.

VPE-only stabil olursa kucuk bir Go To ver:

```text
setspeed 2
```

Go To sirasinda izlenecekler:

```text
mode GUIDED kalmali
EKF_STATUS_REPORT flags 831 civari kalmali
velocity_variance ve pos_horiz_variance buyumemeli
Mode LAND / EKF variance / stopped aiding gelmemeli
```

Eger daha sonra velocity etkisini karsilastirmak istersen bridge'i VPE+VSE ile tekrar dene:

```bash
python3 vision_debug/gazebo_truth_bridge.py \
  --mavlink udp:127.0.0.1:14550 \
  --gz-topic /world/iris_runway/pose/info \
  --model iris_with_gimbal \
  --axis enu \
  --rate 4 \
  --send-speed \
  --speed-source gz \
  --duration 0
```

Bu karsilastirma icin source set 2'de `EK3_SRC2_VELXY=6` gerekir. Ana VPE-only calismada bunu yapma.

Hareket ederken `err=(...)` buyurse once axis'i ayir. Source set 1 GPS'teyken, yani ExternalNav'a gecmeden, bridge'i publish etmeden sadece gozlem modunda calistir:

```bash
python3 vision_debug/gazebo_truth_bridge.py \
  --mavlink udp:127.0.0.1:14550 \
  --gz-topic /world/iris_runway/pose/info \
  --model iris_with_gimbal \
  --observe-only \
  --compare-axes \
  --duration 0
```

Sonra GPS ile kucuk bir hareket yaptir. `axis_err[...] best=...` hangi axis surekli en kucukse bridge'i o `--axis` ile calistir.

## Sonraki calisma - SIFT optimizasyonu

VPE-only Gazebo ground truth testi temiz olmadan bu bolume gecme. Temiz olursa SIFT tarafini EKF'den bagimsiz hizlandiracagiz.

Source set 2 sonrasi telemetry notu:

```text
LOCAL_POSITION_NED artik bagimsiz truth degildir ve bazi MAVLink baglantilarinda
bayatlayabilir. live_sift_nav.py default olarak 1 saniyeden eski telemetry'yi
seed/gate/velocity/truth icin kullanmaz.
```

GPS hala acikken source set 2 debug yapacaksan:

```bash
--telemetry-seed-source global \
--telemetry-max-age-sec 1.0
```

GPS-denied denemede telemetry gate'e guvenme:

```bash
--no-telemetry-position-gate
```

Calisma sirasi:

1. Yeni haritayi net bir map klasorune koy.
2. Haritayi tile'lara bol ve her tile icin metadata tut: bbox, N/E sinirlari, pixel offset, feature sayisi.
3. Tile SIFT descriptor'larini onceden cache'le.
4. GPS/source set 1 ile kamera frame + truth CSV kaydi al.
5. Offline replay benchmark yaz: ayni frameleri localizer'dan gecir, Hz/duration/error/inliers/tile sayisini tabloya dok.
6. ROI algoritmasini benchmark uzerinde ayarla.
7. Basarili ayarlari live sisteme tasi.

ROI hedefi:

```text
predicted_center = last_visual_fix + velocity * dt
search_radius = base_uncertainty + speed * dt + margin
accept -> radius kuculur
reject/miss -> radius kademeli buyur
tile adaylari merkeze uzakliga gore siralanir
```

Performans hedefi:

```text
taze kabul edilen SIFT fix >= 4 Hz
p95 match duration < 0.25 s
p95 accepted error < 2 m at 2 m/s
normalde 1-4 tile arasi arama
```

## Hata notlari

`Address already in use` gorursen 14550 portunu baska bir process dinliyordur. `live_sift_nav.py`, QGC veya baska bir pymavlink script'i kapat.

`HEARTBEAT timeout` gorursen MAVProxy `--out 127.0.0.1:14550` acik degil ya da farkli porta yayin yapiyor.
