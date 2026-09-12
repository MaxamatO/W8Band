# W8Band — wstępna architektura systemu

## Cel urządzenia

Urządzenie zamocowane płasko na środku gryfu sztangi. Łączy się przez BLE z aplikacją
mobilną. Po kalibracji, wykrywa powtórzenia ruchu (na razie: wyciskanie), nagrywa dane
z IMU, przetwarza je lokalnie (edge computing) do postaci trajektorii 2D ruchu gryfu
w płaszczyźnie strzałkowej, i wysyła wynik do telefonu do wizualizacji. W przyszłości:
prędkość, czas trwania faz, inne metryki.

## Ocena wykonalności

**Wykonalne, z zastrzeżeniem dot. dokładności bezwzględnej pozycji.**

Pomiar prędkości z samego IMU to sprawdzona, komercyjnie stosowana metoda (Vmaxpro,
PUSH Band, Beast Sensor, Repone — wszystkie oparte o akcelerometr/IMU). Odtworzenie
*pozycji* (podwójne całkowanie przyspieszenia) jest trudniejsze, bo błąd całkowania
rośnie w czasie — nawet mały bias resztkowy (pojedyncze mg) po dwóch całkowaniach
w kilka sekund daje błąd rzędu metrów.

Kluczowa własność, którą wykorzystujemy: powtórzenie wyciskania jest **ograniczone
w czasie i zaczyna się oraz kończy przy prędkości bliskiej zeru**. To pozwala na
korekcję dryftu metodą ZUPT (zero-velocity update, znaną z nawigacji inercyjnej/
pedestrian dead reckoning) — po każdym powtórzeniu "resetujemy" błąd do zera, więc
błąd nie kumuluje się między powtórzeniami, tylko w obrębie jednego (1-4s).

Realistyczny wynik: wiarygodny **kształt** trajektorii (łuk ruchu, drift boczny,
zawahania w martwym punkcie / sticking point) — nie precyzja co do milimetra
w sensie bezwzględnym. To wystarcza do celu (wizualizacja techniki, porównanie
powtórzeń), ale warto to jasno komunikować użytkownikowi końcowemu, żeby nie
oczekiwał dokładności geodezyjnej.

Jeśli w przyszłości potrzebna będzie twardsza precyzja pozycji absolutnej — jedyna
droga to dodatkowy sensor absolutny (np. ToF/ultradźwiękowy przy stojaku, kamera)
połączony fuzją z IMU. Sam IMU (nawet najlepszy MEMS) tego pułapu nie przebije,
niezależnie od jakości filtrów softwarowych — to fundamentalne ograniczenie fizyki
pomiaru, nie kwestia implementacji.

## Podział odpowiedzialności (warstwy)

1. **FSM (`fsm::StateMachine` + konkretne stany)** — orkiestracja oraz lekki
   obserwator online potrzebny do wykrycia fazy i końca ruchu. Filtr, okna
   sygnałowe i reguły detekcji są wydzielone do helperów; ciężkie i dokładne
   przetwarzanie całego powtórzenia pozostaje poza stanami.
2. **`MotionProcessor`** — zaimplementowana, niezależna od Arduino logika
   przetwarzania sygnału (patrz niżej). Jest bezstanowa i operuje na buforze
   `DataContext::m_Data` przekazanym z zewnątrz. Wywoływana wyłącznie z
   `ProcessingState`.
3. **`DataContext`** — dane współdzielone: bufory (pre-buffer, bufor powtórzenia,
   wynik przetworzenia), parametry kalibracji (bias akcelerometru),
   flagi (np. `m_WakeUpDetected`, docelowo zamiast globalnej zmiennej
   `w8band::v_WakeUpDetected`).
4. **`BleServiceManager` / `LsmServiceManager`** — warstwa sprzętowa, bez zmian
   koncepcyjnych względem tego co już macie.

## Stany FSM

Status połączenia BLE traktujemy jako **osobny wymiar**, nie stan FSM — fizyczny
stan sztangi i status połączenia telefonu to niezależne sprawy.

### 1. Idle

BLE advertising, urządzenie czeka na połączenie i komendę "Kalibruj" z aplikacji.

### 2. Calibration

Wyzwalana komendą z appki, gdy sztanga leży płasko na stojaku (bezruch).

- Pilnuje wariancji modułu przyspieszenia w oknie ~0.5-1s, żeby potwierdzić
  realny bezruch — pojedyncza próbka nie wystarczy. Bieżący `SamplePacket` nie
  zawiera żyroskopu, więc kwaternionu nie wolno traktować jako jego zamiennika.
- Po potwierdzeniu bezruchu liczy bias akcelerometru per oś w układzie ciała
  czujnika, porównując pomiar akcelerometru z wektorem grawitacji SFLP.
- Jeśli wariancja nie spada w rozsądnym czasie (np. 3s) → zgłoś błąd do appki
  zamiast fałszywie potwierdzonej kalibracji.

### 3. Bufferring (stan spoczynkowy / rest)

Po udanej kalibracji: ciągle utrzymuje rolling pre-buffer (`PRE_RECORD_SAMPLES`,
już zaimplementowany) skorygowany o bias, nasłuchuje przerwania wake-up.

- **Rekomendacja:** odświeżaj bias oportunistycznie, gdy sztanga stoi bez ruchu
  dłużej niż np. 3-5s (wolna aktualizacja, np. wykładnicza średnia) — offset MEMS
  dryfuje z temperaturą w trakcie sesji treningowej, jednorazowa kalibracja na
  starcie nie wystarczy na dłuższą metę.

### 4. Armed (rezerwa)

`StateId::ArmedState` jest zarezerwowany, ale nie ma obecnie osobnej
implementacji. `BufferringState` przechodzi bezpośrednio do `RecordingState`,
który w `OnEnter()` zrzuca pre-buffer i inicjalizuje obserwator online. Osobny
stan ma sens dopiero wtedy, gdy dojdą dodatkowe operacje atomowe przed zapisem,
na przykład negocjowanie miejsca w buforze albo jawne uzbrajanie aplikacji.

### 5. Recording

Zapisuje pełną serię próbek do bufora powtórzenia.

- Pre-buffer przechodzi przez ten sam przyczynowy filtr 10 Hz i integrator co
  nowe próbki. Dzięki temu obserwator prędkości startuje z rzeczywistego
  odcinka bezruchu, bez sztucznego stanu początkowego filtra.
- Lekka maszyna faz online potwierdza kolejno: ekscentrykę, koncentrykę,
  hamowanie końcowe, kandydata bezruchu i ogon po zatrzymaniu. Nie wskazuje ona
  dokładnej próbki zawrotu; jedynie uniemożliwia odpalenie końca przed pełną
  sekwencją ruchu dół-góra.
- Kandydat końca łączy historię faz, prędkość pionową, wariancję modułu
  przyspieszenia i RMS pionowego przyspieszenia w świecie. Osobne, szersze
  progi wyjścia tworzą histerezę i zapobiegają migotaniu detekcji.
- Czasy potwierdzania faz i bezruchu są liczone z `timestamp_ticks` sensora.
  `millis()` służy wyłącznie jako niezależny timeout bezpieczeństwa (8 s).
- Po potwierdzonym bezruchu zapisuje jeszcze 300 ms danych dla końcowej kotwicy
  ZUPT, a następnie przechodzi do `ProcessingState`.
- Dokładna zawrotka oraz metryki są liczone dopiero z kompletnego sygnału po
  korekcji dryftu (patrz pipeline niżej).

### 6. Processing

Cała matematyka (opis w sekcji "Pipeline przetwarzania sygnału") odpalona na
kompletnym buforze powtórzenia, przez `MotionProcessor`. Jedyne miejsce, gdzie
dzieje się "ciężkie" liczenie. Dopóki `SendingState` nie jest zaimplementowany,
wynik jest wypisywany przez Serial, zachowywany w `DataContext`, a FSM wraca do
`BufferringState`, aby urządzenie nie zatrzymywało się po pierwszym powtórzeniu.

### 7. Sending

Transmisja **przetworzonej** trajektorii (nie surowego strumienia IMU) przez BLE
— to jest właściwy edge computing: dużo mniej danych do przesłania, mniejsze
obciążenie łącza BLE. Po zakończeniu wraca do Bufferring.

### Error (do dodania)

Osiągalny z każdego stanu: awaria I2C do IMU, rozłączenie BLE w trakcie
nagrywania, przepełnienie bufora. Sygnalizuje problem (LED/BLE) i próbuje
bezpiecznie wrócić do Idle/Bufferring.

## Pipeline przetwarzania sygnału (`MotionProcessor`)

Uruchamiany raz, na kompletnym buforze powtórzenia, w stanie Processing.

1. **Odjęcie biasu** — w układzie ciała czujnika, PRZED rotacją do układu świata
   (bias to zazwyczaj stały offset sprzętowy/elektroniczny sensora, więc jest
   stały w jego własnym układzie odniesienia, nie w układzie świata).
2. **Orientacja** — użyj gotowego game rotation vector z SFLP (już
   zaimplementowane w `LsmServiceManager`), nie pisz własnego filtra
   Madgwicka/Mahony'ego w softwarze — czujnik liczy to sprzętowo, oszczędzasz
   CPU i baterię. Brak magnetometru = brak referencji yaw, ale dla trajektorii
   2D w płaszczyźnie strzałkowej to nieistotne — liczy się tylko przechył
   względem grawitacji, nie obrót wokół osi pionowej.
3. **Usunięcie grawitacji** — obrót wektora przyspieszenia z układu ciała do
   układu świata przez kwaternion orientacji, odjęcie wektora grawitacji
   (0, 0, g) → zostaje przyspieszenie liniowe w układzie świata.
4. **Filtr dolnoprzepustowy** — Butterworth 2. rzędu, domyślne odcięcie 10 Hz,
   wykonany w obu kierunkach (`filtfilt`) na
   przyspieszeniu liniowym PRZED całkowaniem. Szum MEMS mocno się wzmacnia przy
   podwójnym całkowaniu, jeśli nie zostanie wycięty wcześniej.
5. **Całkowanie trapezowe** (albo Simpsona): przyspieszenie → prędkość →
   pozycja, krok czasowy liczony z różnicy `timestamp_ticks` z FIFO sensora
   (`1 tick = 21,75 µs` nominalnie), a nie z czasu odczytu przez MCU.
6. **Korekcja dryftu (ZUPT — zero-velocity update)** — wiadomo, że v(0)=0 oraz
   v(T)=0 (początek i koniec powtórzenia to spoczynek). Policz błąd
   skumulowany na końcu całkowania prędkości i odejmij liniowy trend od całej
   krzywej prędkości tak, żeby oba końce wyszły dokładnie na zero. Usuwa to
   dominujący błąd pochodzący ze stałego biasu resztkowego po kalibracji.
   Prostsze niż pełny filtr Kalmana ze stanem bias+prędkość+pozycja, a dla
   pojedynczego ograniczonego czasowo powtórzenia — wystarczające.
   *(Rozszerzenie na przyszłość: formalny filtr Kalmana z ZUPT jako
   aktualizacją pomiarową — bardziej rygorystyczne, ale znacznie bardziej
   złożone obliczeniowo; raczej niepotrzebne na start.)*
7. **Segmentacja faz ruchu** — na już skorygowanej krzywej prędkości:
   - Rest → Ekscentryka: początek ruchu (przekroczenie progu przyspieszenia/
     prędkości po opuszczeniu spoczynku).
   - Ekscentryka → Zawrotka: przejście prędkości przez zero z ujemnej na
     dodatnią (lokalne minimum pozycji — najniższy punkt ruchu). Dodaj
     histerezę, żeby szum nie generował fałszywych przejść.
   - Zawrotka → Koncentryka: zaraz po najniższym punkcie, gdy prędkość staje
     się trwale dodatnia.
   - Koncentryka → koniec: prędkość wraca blisko zera i utrzymuje się (ten sam
     warunek co koniec nagrywania).
8. **Redukcja wyniku** — współrzędna pozioma jest wyznaczana metodą PCA,
   a trajektoria redukowana domyślnie do 100 punktów `int16` w milimetrach.
   Dokładny punkt zawrotu jest zawsze zachowany w zredukowanym przebiegu.
   `ProcessingResult` przechowuje również ROM, prędkości, czasy faz i — gdy
   skonfigurowano masę sztangi — moc mechaniczną.

## Parametry próbkowania i przepustowości (kontekst)

- 120 Hz (już skonfigurowane, `IMU_FREQ`) — wystarczające dla typowego tempa
  powtórzenia (1-3s).
- Pojedyncze powtórzenie (~4s × 120 Hz = 480 próbek × 24 B `SamplePacket`) =
  ~11,5 KB — mieści się bez problemu w RAM nRF52840 (256 KB), batch processing
  w pamięci jest w pełni wykonalny, nie potrzeba strumieniowania.
- Wysyłanie przetworzonej trajektorii (domyślnie 100 punktów × 4 B jako
  `int16 h,z`) to około 400 B plus metryki — dużo lżejsze niż strumieniowanie
  surowych danych IMU w czasie rzeczywistym, i to jest właściwy sens "edge
  computing" w tym projekcie.

## Dlaczego batch (post-rep), a nie streaming w locie

Korekcja ZUPT wymaga znajomości punktu końcowego (v(T)=0), który nie jest znany
dopóki powtórzenie się nie skończy. W pełni strumieniowe przetwarzanie na żywo
mogłoby dać tylko orientacyjne "czy się ruszam" w czasie rzeczywistym, ale
dokładna, skorygowana trajektoria jest dostępna dopiero po zakończeniu ruchu.
To jest fundamentalny kompromis tej metody, nie ograniczenie implementacyjne —
świadomie budujemy architekturę wokół przetwarzania całego powtórzenia naraz.
