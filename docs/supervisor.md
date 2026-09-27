# Supervisor: etapy 1–3 (rdzeń, panel WWW, kamera, adaptery sprzętu)

Implementacja bazuje na `master` (`7c145d2`), na gałęzi
`feature/system-supervisor`. Rdzeń realizuje pełny cykl na symulowanych
urządzeniach i jest sterowany z CLI albo z panelu w przeglądarce.
Połączenia z robotem, chwytakiem i serwami powstają wyłącznie w jawnym
trybie `--hardware`, a kamera tylko z `--camera`. **Tryb sprzętowy nie był
jeszcze uruchomiony na robocie**; sprawdzono go wyłącznie na atrapach. Detekcja różnicowa nie jest
importowana ani przenoszona do tej gałęzi.

## Uruchomienie

Polecenia uruchamiaj z katalogu głównego worktree supervisora. Kod wymaga
Pythona 3.12 lub nowszego; symulacja korzysta tylko z biblioteki standardowej.

```powershell
python ur5e_experiments/system_main.py --simulate --fast
```

Przykład trzech szklanek i trzech miejsc zakończy się stanem `COMPLETED`,
licznikiem trzech ukończonych sztuk i trzema zajętymi miejscami. `--fast`
przesuwa zegar symulacji; bez tej opcji kroki mają krótkie sztuczne opóźnienia
i można przerwać przebieg przez Ctrl+C.

```powershell
# Pełny odbiór: zakończ pierwszą sztukę i zgłoś WAITING_OUTPUT.
python ur5e_experiments/system_main.py --simulate --fast --glasses 3 --slots 1

# Symuluj operatora opróżniającego jedno miejsce między szklankami.
python ur5e_experiments/system_main.py --simulate --fast --glasses 3 --slots 1 --auto-clear-output

# Zasymuluj błąd gąbki albo naciśnięcie STOP podczas wycierania.
python ur5e_experiments/system_main.py --simulate --fast --fail-at SPONGE
python ur5e_experiments/system_main.py --simulate --fast --stop-at WIPE

# Zapisz zdarzenia do nowego pliku i wypisz tylko końcowy stan.
python ur5e_experiments/system_main.py --simulate --fast --quiet --journal demo-cycle.jsonl
```

Ścieżka `--journal` jest podawana przez operatora, względem bieżącego katalogu
lub jako ścieżka absolutna. Katalog nadrzędny musi istnieć. Istniejący plik nie
zostanie nadpisany. Zdarzenia mają numer, czas monotoniczny, stan, ID partii,
celu i operacji. Końcowy stan jest wypisywany jako JSON.

| Kod wyjścia | Znaczenie |
| --- | --- |
| 0 | Wszystkie szklanki partii obsłużone |
| 1 | Błąd uruchomienia lub zapisu dziennika |
| 2 | Błąd procesu albo nieprawidłowe argumenty CLI |
| 3 | Brak wolnych miejsc odbioru; runner kończy demonstrację w tym stanie |
| 4 | Potwierdzone zatrzymanie przez operatora |
| 5 | Partia rozliczona, ale część celów została pominięta |

## Panel WWW

```powershell
python ur5e_experiments/system_main.py --simulate --web --step-time 0.3
```

Program wypisuje link `http://127.0.0.1:8765/#token=…`. Token trafia do
pamięci sesji karty i zostaje usunięty z paska adresu. Podgląd działa bez
tokenu. START, STOP, RESET, opróżnienie odbioru i dodanie szklanek wymagają
go w nagłówku `X-Supervisor-Token`. Wymagane jest też ciało JSON i zgodny
`Origin`. Domyślnie serwer słucha tylko na `127.0.0.1`. Dostęp z sieci LAN
wymaga jawnego `--host 0.0.0.0`; transmisja HTTP jest nieszyfrowana, więc
token chroni tylko przed przypadkowym sterowaniem. Drugi supervisor na tym
samym porcie nie uruchomi się, bo gniazdo jest otwierane na wyłączność.

Panel pokazuje stan, bieżącą operację, postęp receptury, liczniki partii,
miejsca odbioru z potwierdzaniem opróżnienia, listę szklanek, dziennik zdarzeń
oraz mapę fikcyjnego świata symulacji. Przycisk dodaje szklanki na wolnych
fikcyjnych pozycjach wejściowych. Przy utracie aktualnych danych (powyżej 2 s)
panel oznacza połączenie jako nieaktualne i blokuje przyciski poza STOP.
Zamknięcie przeglądarki nie przerywa partii; ponowne otwarcie odtwarza stan.
Ctrl+C w konsoli zatrzymuje urządzenia i dopiero potem kończy program.

Serwer nie wywołuje metod nadzorcy. `ControlLoop` jest jego jedynym
właścicielem i w każdym takcie wykonuje kolejno: oczekujący STOP, najwyżej
jedno polecenie z kolejki (16 miejsc; przepełnienie daje odmowę), `tick()`
oraz publikację zdarzeń i stanu. STOP ma osobną flagę priorytetową i anuluje
polecenia zakolejkowane przed nim, także START. HTTP 202 oznacza przyjęcie do
kolejki. Wynik nadzorcy zwraca `GET /api/commands/{id}`. START z tym samym
`request_id` nie tworzy drugiej partii. Stan przychodzi przez Server-Sent
Events zamiast WebSocketu: dane płyną tylko do przeglądarki, a polecenia
i tak idą przez HTTP. Wolny klient blokuje wyłącznie swój wątek.
Nie ma zależności poza biblioteką standardową Pythona.

| Interfejs | Działanie |
| --- | --- |
| `GET /api/status` | Pełny stan, także po odświeżeniu |
| `GET /api/events` | SSE: zdarzenia nadzorcy i stan co 0,5 s |
| `GET /api/commands/{id}` | `queued`, `done` (z odpowiedzią nadzorcy), `cancelled` |
| `GET /camera.mjpg` | Ostatnia klatka z opisem detekcji (tylko `--camera`) |
| `POST /api/batch/start` | `{"request_id": "…"}` |
| `POST /api/stop` | Priorytetowe zatrzymanie |
| `POST /api/fault/reset` | Kontrolowany powrót do `READY`, bez ruchu |
| `POST /api/output/confirm-cleared` | `{"slot_ids": ["tag-1"]}` |
| `POST /api/sim/add-glasses` | `{"count": 3}`, tylko fikcyjne cele |

## Kamera: klasyczna detekcja z `master`

```powershell
python ur5e_experiments/system_main.py --simulate --web --camera
```

Szklanki pochodzą z kamery nad stołem. Robot, chwytak i stanowiska pozostają
symulowane. Wymaga OpenCV, NumPy i plików kalibracji `find_glasses.py`
(`overhead_camera_calibration.npz`, `overhead_camera_pose.npz`). Indeks
kamery, rozdzielczość i obszar detekcji (`detection_area.json`) pochodzą
z `find_glasses.py`. Obszar musi obejmować tylko strefę wejściową, bez
stanowisk i tagów odbioru. Szklanki muszą stać otworem do góry.

`system_vision.py` uruchamia kamerę w **osobnym procesie** (`multiprocessing`,
`spawn`). Proces kamery otwiera kamerę i kalibrację, wykrywa szklanki i przez
potok wysyła tylko gotowe pomiary i podgląd JPEG, więc przetwarzanie obrazu
nie konkuruje z pętlą sterowania o GIL. Błąd otwarcia kamery przerywa start
programu. Gdy proces kamery zginie, pomiar się zestarzeje, a nadzorca
zatrzyma partię. Kamera Z klatek w oknie 0,3 s
tworzy jeden pomiar: szklanka musi być wykryta w co najmniej połowie
klatek okna. Mniej niż 5 klatek w oknie nie daje pomiaru; obraz staje się
nieaktualny, a nadzorca zatrzymuje partię. Dzięki temu trzy kolejne pomiary
bez celu oznaczają około 1 s nieobecności, a nie trzy klatki. ID szklanki
przechodzi na kolejne okno, jeżeli przesunęła się o najwyżej 3 cm. Większy
skok daje nowe ID, a stary cel zostanie pominięty. Niejednoznaczne
dopasowania są pomijane w danym pomiarze. Nadzorca nie odczytuje kamery
i nie czeka na nią; `observe()` zwraca ostatni opublikowany pomiar.

Na prawdziwym stole podniesiona w symulacji szklanka nadal stoi. Bieżąca
partia rozlicza ją raz; następny START obejmie ją ponownie. Ten tryb nie
był jeszcze uruchomiony z kamerą: przetestowano go wyłącznie offline.

## Tryb sprzętowy

Wymagana jest dokładnie jedna z flag `--simulate` / `--hardware`.

```powershell
# Uczenie pozycji (bez partii): freedrive, chwyt/zwolnienie, zapis pozy.
python ur5e_experiments/system_main.py --hardware --web --teach
# Praca: szklanki z kamery, reszta sprzętowa.
python ur5e_experiments/system_main.py --hardware --web --camera
```

`--hardware` łączy UR5e przez `safe_motion.connect()`, otwiera port chwytaka
i magistrali serw, a watchdog RTDE uzbraja tuż przed startem pętli. **Od tej
chwili ramię może się ruszać.** Otwarcie portu XIAO może zresetować
mikrokontroler, więc przy starcie chwytak nie może trzymać szklanki. Porty
nie są automatycznie otwierane ponownie. START jest odrzucany (powód widać
w panelu), gdy brakuje nauczonej pozy lub pomiaru stanowiska, zmienił się
offset TCP, TCP ma przesunięcie x/y, freedrive jest włączony albo ramię stoi
dalej niż 5 cm od pozy `observe` (kamera musi widzieć stół). Braki
konfiguracji blokują tylko START; RESET po błędzie pozostaje możliwy.

- **Zajętość miejsc odbioru** jest zapisywana w `system_outputs.json` po
  każdej zmianie. Restart nigdy nie zwalnia miejsca: `RESERVED` wraca jako
  `UNKNOWN`, a po nieczystym zakończeniu (awaria, partia w toku) także `FREE`
  wraca jako `UNKNOWN`. Brak lub uszkodzenie pliku oznacza wszystkie `UNKNOWN`.
- **Jedna instancja**: `supervisor.lock` jest blokowany na czas działania
  procesu; system zwalnia go także po awarii.
- **Dziennik**: zdarzenia zawsze trafiają do `logs/supervisor-<data>.jsonl`
  (albo do pliku z `--journal`).
- **`--gamepad`**: dowolny ruch drążka podczas partii wysyła STOP; poza
  partią drążki przesuwają ramię (`gamepad_jog.Jogger`, sufit `SafeControl`
  i `MIN_TCP_Z`), np. do pozy `observe`.
- **OBSERVE** kończy się dopiero pomiarem kamery, którego okno zaczęło się
  po dojeździe ramienia; żadna klatka tego pomiaru nie pokazuje ramienia.

### Co robi adapter (`system_hardware.py`)

Każdy krok receptury to plan akcji zbudowany z nauczonych póz i sprawdzony
geometrycznie przed pierwszym ruchem. Warunki zakończenia są sprawdzane:

| Krok | Plan | Warunek powodzenia |
| --- | --- | --- |
| PICK | przejazd nad strefę, podejście do punktu 3 cm przed ścianką, GRIP, ruch kontaktowy wzdłuż osi przyssawki (≤ 3,6 cm, 5 N) | kontakt siłowy i `GRIP OK` z żywym `HOLD YES` w 6 s |
| LIFT | pionowo na wysokość przejazdu | poza osiągnięta |
| FLIP | przejazd do pozy `flip`, obrót nadgarstka 3 o 180° (`moveJ`) | kąty osiągnięte, TCP nie przesunął się |
| TO_/LEAVE_ stanowisk | podejście → praca; praca → podejście → wysokość przejazdu | pozy osiągnięte, drogi w korytarzach |
| SPRAY_1/2 | 3 / 2 skoki pompki | każda pozycja serwa potwierdzona |
| SPONGE | profil pozycji serwa gąbki | j.w., bez błędów statusu |
| WIPE | nauczone pociągnięcia z `WIPE_SPEED` | siła ≤ `WIPE_MAX_FORCE`, pozy osiągnięte |
| TO_OUTPUT, LOWER | nad pozę odbioru + 2 cm; ruch kontaktowy w dół (≤ 3,5 cm, 8 N) | **kontakt**; koniec drogi to błąd, bez RELEASE |
| RELEASE | impuls zwolnienia | sterownik wraca do `IDLE` (koniec impulsu) |
| RETREAT, OBSERVE | odsunięcie wzdłuż osi przyssawki, w górę; poza obserwacji | pozy osiągnięte |

Ruchy liniowe są asynchroniczne. Ruch kontaktowy to `speedL` z czasem
0,02 s odświeżany co takt. STOP unieważnia zadania urządzeń, wysyła
`speedStop` i asynchroniczne `stopL`/`stopJ`, zatrzymuje serwa (pompka
w spoczynek) i czeka na potwierdzenie; nigdy nie wyłącza podciśnienia.
`ControlLoop.service()` kopie watchdog RTDE w każdym takcie, również podczas
zatrzymywania przy zamykaniu, i sprawdza sufit TCP.

Prędkości, siły kontaktu, odstępy i czasy chwytu pochodzą z
`pick_place_glasses.py`. Nowe wartości w `system_hardware.py` są oznaczone
„TO BE MEASURED”: `FLIP_SPEED`, `WIPE_SPEED`, `WIPE_MAX_FORCE`, liczby skoków
pompki, profil gąbki i tolerancje. Limity Z pochodzą z `SafeControl`
i `follow_april_tag`, nigdy z pliku układu. `PICK_TIMEOUT` nadzorcy wzrósł
z 10 do 30 s, bo obejmuje teraz dojazd, ruch kontaktowy i potwierdzenie
chwytu. To termin operacji, nie limit ruchu.

### Kontrola całego ramienia (`system_arm.py`)

Przed wysłaniem planu adapter odtwarza przebieg kątów stawów od aktualnej
konfiguracji robota. Dla `moveL` robi to kinematyką odwrotną co 1 cm lub
0,05 rad, rozwiązywaną w pobliżu poprzedniej próbki, tak jak sterownik.
Dla obrotu nadgarstka (`moveJ`) interpoluje liniowo kąty. Plan jest
odrzucany przed ruchem, gdy:

- pozy nie da się osiągnąć po linii prostej;
- między próbkami następuje skok stawów (osobliwość, zmiana konfiguracji);
- przekroczony zostałby limit stawu;
- ramię, przedramię, nadgarstek albo korpus chwytaka (kapsuły wzdłuż
  łańcucha DH) zbliżają się do bryły stanowiska na mniej niż promień
  członu plus `clearance`.

Nadgarstek i chwytak mogą wejść w bryłę aktywnego stanowiska tylko wewnątrz
jego korytarza, tak jak narzędzie. Walidacja długiej trasy trwa dłużej niż
okres watchdoga, więc w trakcie liczenia watchdog jest kopany.

Ograniczenia modelu: nominalne parametry DH UR5e (bez kalibracji
konkretnego robota), szacunkowe promienie członów i przesunięcie barku
(„TO BE MEASURED”), bez podstawy, barku, stołu i przeszkód spoza modelu.
Przy każdej próbie START model jest porównywany z pozą TCP raportowaną
przez robota. Rozbieżność powyżej 5 mm lub 1° blokuje START. Ostateczną
ochroną pozostają ustawienia bezpieczeństwa robota (płaszczyzny, limity
stawów).

### Plik uczenia `system_teach.json`

Pozy zapisuje panel (`--teach`, poza partią, robot nieruchomy). Pozy
stanowisk uczy się ze szklanką w chwytaku, otworem w dół. Offset TCP
zapisany przy uczeniu musi zgadzać się z robotem. Wymiary szklanki
i stanowisk operator wpisuje ręcznie po pomiarze:

```json
{
  "version": 1,
  "tcp_offset": [0, 0, 0.2, 0, 0, 0],
  "poses": {"observe": {"pose": [0.3, -0.5, 0.35, 0, 1.571, 0], "q": [0, -1.57, 1.57, -1.57, -1.57, 0], "captured": "..."}},
  "glass": {"radius": 0.03},
  "layout": {
    "tool_radius": 0.12, "clearance": 0.03,
    "stations": {
      "sprayer": {"low": [0.2, 0.25, 0.0], "high": [0.4, 0.45, 0.2],
                  "corridor": {"low": [0.19, 0.04, 0.04], "high": [0.41, 0.41, 0.26]},
                  "solid": [{"low": [0.28, 0.4, 0.0], "high": [0.32, 0.45, 0.2]}]},
      "sponge": {"...": "..."}, "wiper": {"...": "..."}
    }
  }
}
```

Wymagane pozy: `observe`, `side_grip` (przyssawka na ściance stojącej
szklanki, narzędzie poziomo; używane są orientacja i wysokość), `flip`,
`<stanowisko>.approach` i `.work` dla `sprayer`, `sponge`, `wiper`,
`wiper.stroke.1..N` (kolejno) oraz `output.<miejsce>` (TCP, gdy odwrócona
szklanka stoi na miejscu odbioru). Z (w metrach) jest w układzie bazy.
`tool_radius` musi objąć chwytak ze szklanką w każdej orientacji.

### Pierwsze uruchomienie na sprzęcie (do zrobienia przy robocie)

1. `bus_servos.py scan`: potwierdzić ID serw (`SPRAYER_ID` = 1, gąbka =
   `ROTATOR_ID` = 2) i kierunki; profil gąbki ustawić w `SPONGE_POSITIONS_DEG`.
2. Na pendancie sprawdzić offset TCP (bez x/y) i płaszczyznę bezpieczeństwa.
3. `--hardware --web --teach` bez szklanki: STOP w panelu, zamknięcie
   przeglądarki (partia trwa), Ctrl+C (zatrzymanie urządzeń).
4. Zmierzyć stanowiska i szklankę, nauczyć poz, wpisać układ.
5. Pojedyncze kroki przy zmniejszonej prędkości na pendancie, ręka na
   zatrzymaniu awaryjnym; dopiero potem pełny cykl jednej szklanki.

Symulator odwzorowuje kolejność i stan operacji oraz sprawdza drogi TCP
względem przykładowych brył stanowisk, z uwzględnieniem otoczki narzędzia
i szklanki. Nie symuluje członów ramienia, podciśnienia fizycznego, kontaktu
z gąbką ani rzeczywistego wykonania ruchu. Jego wymiary są fikcyjne.

## Zaimplementowane zachowanie

- Jedna aktywna partia i jedna aktywna operacja. Ponowiony START z tym samym
  ID zwraca pierwotne potwierdzenie bez ponawiania ruchu. Inny START podczas
  pracy zostaje odrzucony. Identyfikatory są pamiętane przez czas działania
  procesu; po zapełnieniu limitu nowe START są odrzucane.
- Cykl: chwyt → podniesienie → obrót → sprysk → gąbka → drugi sprysk →
  wycieranie → opuszczenie na podparcie → zwolnienie → odsunięcie → obserwacja.
  Przejazdy między stanowiskami oraz wycofanie po każdym stanowisku są
  odrębnymi operacjami. Dopiero ukończone wycofanie pozwala jechać dalej.
- Rezerwacja miejsca przed chwytem. Brak miejsca przełącza na `WAITING_OUTPUT`
  z pustym chwytakiem. `confirm_output_cleared()` pozwala kontynuować tę samą
  partię po potwierdzonym opróżnieniu odbioru.
- Cel ponownie mierzony przed chwytem; nowo dodane szklanki nie wchodzą do
  bieżącej partii. Brak celu w trzech nowych obserwacjach oznacza pominięcie.
  Symulator dostarcza stabilne ID; dopasowanie detekcji z rzeczywistej kamery
  będzie odpowiedzialnością adaptera wizji.
- Świeżość kamery i telemetrii sprawdzana przez cały aktywny cykl. Powtórzone
  odczytanie tej samej klatki nie jest nowym pomiarem. Telemetria ma czas
  najstarszej składowej; odczyt danych z pamięci nie może odmładzać próbki.
- Podniesienie wymaga `Grip.OK`. Utrata lub nieznany stan chwytu zatrzymuje
  także oczekiwanie przy spryskiwaczu/gąbce. Osiągnięcie limitu opuszczania
  bez potwierdzonego podparcia nie pozwala zwolnić szkła.
- STOP anuluje komendy i pozostawia podciśnienie. `STOPPING` trwa do
  potwierdzenia zatrzymania robota oraz napędów stanowisk; timeout daje `FAULT`.
  Spóźniona odpowiedź dawnej operacji nie uruchamia kolejnego etapu.
- Przerwana rezerwacja staje się `UNKNOWN`. Już zajęte miejsce pozostaje
  zajęte, także przy STOP podczas odsuwania ramienia po odłożeniu.
- RESET wymaga potwierdzonego zatrzymania, pustego chwytaka, sprawnych urządzeń
  i rozstrzygniętej zajętości odbioru. Przywraca tylko `READY`; nowy START ma
  nowe ID polecenia. Powrót komunikacji nie wznawia przerwanej operacji.

## Wysokość i obrys stanowisk

`system_geometry.py` przechowuje przestrzenny obrys każdego stanowiska:
przedziały X/Y/Z w metrach względem bazy robota, opcjonalny nauczony korytarz
roboczy oraz części sztywne, których nie wolno dotknąć również podczas pracy.
Górna granica obejmuje najwyższy punkt osprzętu i cały zakres ruchu serwa.
Wysokość zmierzona od stołu musi zostać przeliczona przez dodanie `table_z`.

Sprawdzana objętość narzędzia to kula wokół TCP mieszcząca chwytak i szklankę
w każdej orientacji, również w trakcie obrotu. Do obliczeń dodawany jest
margines. Test odcinka względem powiększonej bryły jest konserwatywny:
może odrzucić wąskie przejście, ale nie polega na samym sprawdzeniu końców.
Dotknięcie granicy powiększonej przeszkody także odrzuca ruch.

Domyślna trasa podnosi TCP ponad najwyższe stanowisko z zapasem na całą
otoczkę i margines, następnie przejeżdża poziomo i schodzi do celu.
Sprawdzane są wszystkie odcinki, w tym wznoszenie i schodzenie. Jeśli wymagana
wysokość przekracza limit TCP, plan zostaje odrzucony przed wysłaniem ruchu.
Walidator potrafi także sprawdzić zadaną trasę bokiem; automatyczny planer
objazdów nie jest zaimplementowany.

Korytarz roboczy własnego stanowiska jest dostępny wyłącznie na końcowym
odcinku podejścia, podczas odpowiedniej operacji roboczej i pierwszym odcinku
wycofania. Musi pomieścić całą otoczkę. Wycofanie musi kończyć się poza
stanowiskiem. Pozostałe stanowiska i części sztywne są sprawdzane również
w tych etapach. Nie ma ogólnej flagi wyłączającej kontrolę danego stanowiska.

Przed `begin()` nadzorca wywołuje `validate_motion()`. Symulator zachowuje
dokładnie sprawdzoną trasę; zmieniona komenda, pozycja początkowa lub geometria
unieważnia przygotowany ruch. Odmowa walidacji uruchamia ścieżkę zatrzymania.

`example_workspace()` zawiera trzy **fikcyjne** stanowiska o różnych
wysokościach, przeznaczone wyłącznie do demonstracji i testów. Przed integracją
sprzętową potrzebne są pomiary obrysów, wysokości, osprzętu, otoczki narzędzia
i zatwierdzenie korytarzy. Trzeba również sprawdzić geometrię wszystkich
członów UR5e i rzeczywistą trajektorię. Ten walidator nie sprawdza łuków
`moveJ`, przewodów, ludzi ani przeszkód nieujętych w modelu. Limity prawdziwego
adaptera muszą pochodzić z konfiguracji `SafeControl`, bez ich zwiększania.

## Podział kodu

| Moduł | Zadanie |
| --- | --- |
| `system_main.py` | CLI, zegar symulacji/czas rzeczywisty, zapis zdarzeń |
| `system_controller.py` | Maszyna stanów, START/STOP/RESET, postęp i kontrola warunków |
| `system_operations.py` | Receptura cyklu, timeouty i nieblokujący kontrakt adaptera |
| `system_batch.py` | Cele partii, ich wyniki i rezerwacje odbioru |
| `system_model.py` | Typy poleceń, odpowiedzi, telemetrii i obserwacji |
| `system_simulator.py` | Logiczne urządzenia i wstrzykiwanie błędów |
| `system_geometry.py` | Bryły stanowisk, otoczka narzędzia, kontrola całych odcinków i wysokości przejazdu |
| `system_settings.py` | Parametry czasowe, bez nowych limitów ruchu fizycznego |
| `system_web.py`, `web/` | Pętla właściciela nadzorcy, kolejka poleceń, serwer HTTP/SSE/MJPEG, panel |
| `system_vision.py` | Okna pomiarowe, stabilne ID szklanek, wątek kamery z `GlassFinder` |
| `system_hardware.py` | Adapter UR5e: plany kroków, warunki końcowe, STOP, watchdog, uczenie |
| `system_io.py` | Wątki właścicieli portów chwytaka i magistrali serw |
| `system_teach.py` | Nauczone pozy, pomiary układu, kontrola kompletności, zapis atomowy |
| `system_arm.py` | Kinematyka UR5e, przebieg stawów planu, kolizje członów ze stanowiskami |

Nadzorca ma jednego właściciela: `ControlLoop`. Panel przekazuje mu komendy
kolejką; nie wolno wywoływać metod nadzorcy równolegle z kilku wątków. `DeviceAdapter`
określa rozpoczęcie/polling operacji, obserwację, telemetrię i oddzielne
potwierdzenie STOP oraz walidację dokładnej planowanej drogi. Implementacje sprzętowe muszą zwracać szybko i weryfikować
warunki fizyczne, zamiast zgłaszać sukces po wysłaniu komendy.

## Weryfikacja i następny etap

```powershell
python -m pytest ur5e_experiments/tests hoverboard_experiments/tests -q
```

Nowe testy obejmują pełną partię, STOP i awarię w każdym etapie, utratę
chwytu podczas pracy stanowisk, brak podparcia, timeouty i spóźnione odpowiedzi,
pełny odbiór, nieaktualne klatki, idempotencję START, reset, CLI i dziennik.
Testy geometrii sprawdzają również przecięcie stanowiska pomiędzy wolnymi
końcami ruchu, otoczkę podczas obrotu, granicę marginesu, pionowe odcinki,
limity wysokości, dostęp do korytarzy i blokowanie niezweryfikowanego ruchu.

Testy panelu obejmują: kolejność STOP przed poleceniami, anulowanie
zakolejkowanego START, idempotencję, przepełnienie kolejki, odmowę błędnych
akcji, token, `Origin`, typ treści, SSE, MJPEG, zajęty port i zatrzymanie
urządzeń przy zamykaniu. Serwer HTTP w testach działa na porcie efemerycznym
w pętli zwrotnej. Testy wizji sprawdzają okna, mediany, odrzucanie odbić,
utrzymanie i zmianę ID, niejednoznaczność, limit śledzonych celów oraz
zewnętrzne źródło sceny symulatora. Kamera nie jest otwierana w testach.

Testy sprzętowe na atrapach (`tests/fake_robot.py`): pełny cykl ze
sprawdzeniem wszystkich warunków końcowych, brak kontaktu przy chwycie
i odkładaniu (bez RELEASE), niepotwierdzony chwyt (podciśnienie zostaje),
ruch zakończony poza celem, błąd serwa, STOP z asynchronicznym `stopL`,
zadziałanie watchdoga, ruch robota między walidacją a startem, droga przez
stanowisko, niekompletny plik uczenia, zmieniony offset TCP, freedrive,
zapis pozy tylko poza partią. Wątki I/O: mapowanie stanu chwytu,
potwierdzenie końca impulsu zwolnienia i jego brak, STOP bez zwolnienia,
utrata USB, potwierdzanie pozycji serw, zablokowane serwo, błąd statusu,
anulowanie sprysku i niepotwierdzone zatrzymanie.

Etap 3 w części offline jest zrobiony. Pozostaje praca przy robocie
(lista wyżej), potem etapy 4–7 [planu](system_supervisor_plan.md).
Panel pozostaje w procesie nadzorcy, w wątkach. Serwer HTTP głównie czeka
na sieć. Zmierzony najdłuższy takt pętli przy 8 otwartych strumieniach SSE
i pełnej partii w symulacji wyniósł 7,8 ms, wobec okresu 20 ms i 200 ms
watchdoga. Pomiar trzeba powtórzyć na docelowym laptopie z kamerą; status
pokazuje go jako `loop.max_tick`.

Otwarte punkty: próba `--camera` na stole; pomiar promieni członów
i przesunięcia barku; pomiar czasów pętli na docelowym laptopie. Nowe uruchomienie symulatora tworzy nowy fikcyjny świat;
dziennik nie służy do automatycznego wznawiania ruchu.
