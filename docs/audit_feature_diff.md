# Audyt feature/table-diff-classifier

Data: 2026-09-27. Badany commit: `887bbc3`.
Porównanie: `master...HEAD`, wspólny przodek `7c145d2`.
W repozytorium nie ma brancha `feature/diff`; audyt dotyczy aktualnego
`feature/table-diff-classifier` i jego pięciu commitów.

## Wynik

Znaleziono trzy potwierdzone błędy: dwa P1 (do poprawienia przed użyciem
automatycznego chwytu) i jeden P2. Istniejący zestaw testów przechodzi,
ale nie obejmuje tych przypadków. Nie zmieniono kodu wykonawczego.

### 1. P1 — filtrowanie obszaru usuwa przeszkody blokujące chwyt

Miejsce: `ur5e_experiments/find_glasses.py:485–490`, użycie w linii 471;
skutek w `pick_place_glasses.py:504–513`.

`_object_in_area()` bada wyłącznie wybrane wierzchołki konturu przeszkody.
Nie jest to test przecięcia wielokątów. Długi prostokąt może przecinać cały
obszar detekcji, a jednocześnie mieć wszystkie cztery wierzchołki poza nim.
Analogicznie pomijana jest przeszkoda otaczająca obszar. Próbkowanie co n-ty
wierzchołek dodatkowo może pominąć część konturu wchodzącą do obszaru.

Reprodukcja offline, współrzędne w metrach:

- obszar: prostokąt x=0.3–0.5, y=0–0.2;
- przeszkoda `obstruction`: prostokąt x=0.2–0.6, y=0.08–0.12;
- szklanka `DOWN`: środek (0.4, 0.15), średnica 0.06.

Wynik `_object_in_area()`: `False`. Przekazanie przeszkody bezpośrednio do
`pickable()` daje `obstruction next to it`; po filtrowaniu wynik to `None`,
czyli zgoda na chwyt. Odległość osi od przeszkody wynosi 3 cm, przy wymaganym
zasięgu kontroli 9 cm. Nie zależy to od jakości klasyfikacji obrazu.

Ponadto nawet prawidłowy test przecięcia nie rozwiąże przypadku przeszkody
tuż poza obszarem, ale w odległości mniejszej niż wymagany odstęp od szklanki
wewnątrz. Obszar wyboru szklanek nie powinien przycinać danych dla kontroli
odstępu.

Zalecenie: do kontroli chwytu zachować wszystkie wykryte przeszkody albo
filtrować je względem obszaru powiększonego o wymagany odstęp, z poprawnym
testem geometrycznym. Filtrowanie wizualizacji może pozostać osobne.
Testy regresyjne: przecięcie bez wewnętrznych wierzchołków, zawieranie obszaru,
przeszkoda tuż za granicą oraz kontur z dużą liczbą punktów.

### 2. P1 — zastępczy okrąg zamienia płaski kwadrat w szklankę do chwytu

Miejsce: `ur5e_experiments/glass_classifier.py:415–421`.

Jeżeli nie zaakceptowano żadnej szklanki, kod traktuje minimalny okrąg
obejmujący plamę jako zaobserwowany koniec szklanki. Sprawdza tylko zakres
promienia, bez wymagania rzeczywistej okrągłej krawędzi lub odpowiedniej
kolistości. Hipoteza otrzymuje potem normalną orientację i może być chwytana.

Reprodukcja pełnego toru obraz → maska → klasyfikacja, przy domyślnych
parametrach testowej kamery i szklanki z `tests/synthetic_table.py`:

1. Zapamiętać pustą matę przez `TableBackground.capture(..., frames=5)`.
2. Na kopii maty narysować jasny wypełniony kwadrat:
   `cv2.rectangle(scene, (400, 300), (444, 344), (220, 220, 220), -1)`.
3. Wykonać `st.photo(scene)`, `background.compare()` i `gc.classify()`.

`find_circles()` zwraca pustą listę. Mimo to wynik klasyfikacji to jedna
szklanka `DOWN`, koszt około 3.19, margines około 8.51, bez obiektów
`unknown`. Kwadrat ma około 4.9 cm boku na płaszczyźnie stołu.
W 15 kolejnych klatkach z niezależnym szumem i włączoną adaptacją wynik
pozostaje `DOWN` w 15/15 klatek. Głosowanie orientacji nie usuwa błędu.
Podobny kwadrat o boku 52 px daje `UP`; domyślny chwyt boczny dopuszcza
również tę orientację.

Skutek: nowy przedmiot na stole, bez żadnej wykrytej okrągłej krawędzi,
może trafić na listę kandydatów do automatycznego chwytu. Usunięcie jego
obszaru jako wyjaśnionego przez szklankę dodatkowo usuwa go z listy obiektów.

Zalecenie: nie nadawać zgody na chwyt na podstawie samego okręgu obejmującego
plamę. Wymagać niezależnego potwierdzenia geometrii; w przeciwnym razie
zachować `unknown` lub niepewną hipotezę. Sama zmiana progu marginesu nie
rozwiązuje źródła problemu.
Testy regresyjne: kwadraty, prostokąty i płaskie krążki o rozmiarach zbliżonych
do szklanki, w kilku miejscach obrazu, również w sekwencji wielu klatek.

### 3. P2 — pokrycie modelu liczone tylko w przyciętym oknie zawyża dopasowanie

Miejsce: `ur5e_experiments/glass_classifier.py:209–214`.

`Blob.coverage()` rasteruje przewidywany obrys na tablicy o rozmiarze okna
plamy. Fragmenty obrysu wykraczające poza okno znikają zarówno z licznika,
jak i mianownika. Metryka ignoruje więc część modelu, której w obrazie nie
potwierdzają zmienione piksele. Ma to znaczenie zwłaszcza dla pojedynczego
okręgu: paralaksa przewidywanego drugiego końca może przekraczać 10 px
`ROI_PADDING`.

Reprodukcja: maska prostokąta (620,340)–(660,380), przewidywany obrys
(600,320)–(680,400). Funkcja zwraca 0.45176, podczas gdy pokrycie całego
obrysu wynosi 0.25621.

Sprawdzono także wpływ na decyzję klasyfikatora: pełny dysk odpowiadający
górnemu końcowi `DOWN` w punkcie (0.72,-0.12), bez potwierdzenia drugiego
końca, daje zaakceptowaną hipotezę `?`, koszt 2.63. Po zmianie wyłącznie
mianownika na pełny obrys kandydat jest odrzucany (`None`). W tym przykładzie
`?` nie dopuszcza chwytu, ale błędna akceptacja zmienia listę szklanek i
usuwanie obiektów niewyjaśnionych przez model.

Zalecenie: mianownik powinien obejmować cały przewidywany obrys; licznik
powinien liczyć przecięcie z maską plamy. Osobno określić politykę dla
modeli wychodzących poza sam obraz. Test powinien porównywać wynik z
referencyjną rasteryzacją całego obrysu i obejmować przesunięcie paralaksy
większe od marginesu okna.

## Weryfikacja i granice audytu

- Przejrzano wszystkie zmiany brancha: model tła, klasyfikator, integrację
  detekcji i chwytu, nowe testy oraz dokumentację.
- `git diff --check master...HEAD`: bez uwag.
- `python -m pytest ur5e_experiments/tests -q -p no:cacheprovider
  --basetemp=./.audit-venv/pytest-tmp`: **105 passed in 21.83s**.
- Środowisko audytu: Python 3.14, NumPy 2.5.3, OpenCV 5.0.0.93, pytest 9.1.1.
  Utworzono lokalne, osobne środowisko `.audit-venv`; dokumentacja projektu
  wskazuje Python 3.12, więc nie jest to walidacja tego dokładnego środowiska.
- Reprodukcje używały syntetycznych obrazów, czystych funkcji i zastępników
  bibliotek sprzętowych z istniejącego `conftest.py`. Nie uruchamiano kamery,
  połączeń z robotem ani sterowników chwytaka.
- Nie zweryfikowano na nagraniach rzeczywistych szklanek jakości przy
  refleksach, częściowych zasłonięciach, stykających się szklankach i pochyleniu
  kamery. Istniejące sceny syntetyczne zakładają kamerę skierowaną pionowo w dół.
- Nie zmierzono budżetu czasu detekcji względem watchdoga 0.2 s na docelowym
  komputerze i obrazach z rzeczywistej kamery; przejście testów nie stanowi
  potwierdzenia pracy w czasie rzeczywistym.

Priorytet napraw: zachowanie przeszkód przy wyborze chwytu, ograniczenie
zastępczego okręgu, poprawa metryki pokrycia. Każda naprawa powinna dostać
test przypadku negatywnego, a następnie walidację na zapisanych klatkach
z rzeczywistej kamery przed sprawdzeniem ruchu.

## Uzupełnienie: C207A0 bezpośrednio po uruchomieniu skryptu

Operator zgłosił `Protective Stop — C207A0: Fieldbus input disconnected`
natychmiast po uruchomieniu skryptu. Najbardziej prawdopodobna przyczyna
w badanym kodzie to przekroczenie czasu między aktualizacjami watchdoga
RTDE przez synchroniczne przetwarzanie obrazu. Bez logu zdarzenia nie można
wykluczyć utraty połączenia ani watchdoga innego aktywnego interfejsu.

`robot_watchdog.py:36` ustawia 5 Hz, czyli nominalny okres 200 ms.
`pick_place_glasses.py:562` włącza watchdog przed wejściem do pętli.
Pierwszy `kick()` następuje dopiero po odczycie i korekcji klatki (linia 570).
Następnie detekcja szklanek, AprilTagów, rysowanie i obsługa okna wykonują
się w tym samym wątku, bez aktualizacji watchdoga w tych operacjach.
Analogiczny układ ma `find_glasses.py --robot`.

Pomiary offline po zgłoszeniu błędu, bez połączenia z robotem:

- Tryb `classic`, lokalne ustawienia i kalibracja, syntetyczna klatka
  1280×720: pierwsze wywołanie korekcji, detekcji, AprilTagów i nakładki
  trwało łącznie 202.5 ms. Składowe: 7.9 / 93.8 / 26.6 / 74.2 ms.
  Kolejne iteracje trwały 124.5–142.5 ms. Pomiar nie obejmował oczekiwania
  na kamerę, `imshow`, obsługi klawiszy ani połączeń RTDE. Nie jest to
  bezpośredni pomiar odstępów pakietów na robocie.
- Tryb `diff`, siedem syntetycznych szklanek: samo porównanie tła i
  klasyfikacja zajmowały w dziesięciu pomiarach 104.5–395.4 ms;
  dwa wywołania przekroczyły 370 ms. Wewnątrz tej operacji nie ma `kick()`.
- W tym checkoutcie nie ma `table_background.npz`; konstruktor wybiera
  `classic`. Problem rozruchu nie wymaga więc uruchomienia trybu `diff`.

Pomiary potwierdzają brak zapasu czasowego i możliwość przekroczenia
budżetu watchdoga, ale nie odtwarzają samego zdarzenia z robota. To odrębna
kwestia od trzech błędów klasyfikacji opisanych wyżej; zależność watchdoga
od pętli obrazu istniała już przed tym branchem.

Kierunek naprawy: przygotować pierwszy obraz i zainicjalizować kosztowne
operacje przed uzbrojeniem watchdoga; oddzielić długie operacje wizyjne
od terminowej obsługi sterowania, z kontrolą świeżości wyników i zatrzymaniem
przy utracie postępu. Sam niezależny wątek wysyłający `kick()` bez kontroli
postępu pętli sterowania maskowałby jej zawieszenie. Samo obniżenie
częstotliwości watchdoga zmieniłoby czas reakcji zabezpieczenia.

Potwierdzenie przyczyny konkretnego zdarzenia wymaga korelacji logu robota
z czasami `setWatchdog`/`kickWatchdog` i etapów pierwszej iteracji programu.
Nie zmieniano ustawień bezpieczeństwa ani kodu sterowania.

Źródła: [UR — C207](https://www.universal-robots.com/manuals/EN/HTML/SW10_7/Content/prod-err-codes/topics/CODE_207.html),
[UR — rtde_set_watchdog](https://www.universal-robots.com/manuals/EN/HTML/SW10_14/Content/prod-scriptmanual/all_scripts/rtde_set_watchdog_variable_name.htm).
