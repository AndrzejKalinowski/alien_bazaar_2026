# Supervisor: etapy 1–2 (rdzeń, panel WWW, kamera)

Implementacja bazuje na `master` (`7c145d2`), na gałęzi
`feature/system-supervisor`. Rdzeń realizuje pełny cykl na symulowanych
urządzeniach i jest sterowany z CLI albo z panelu w przeglądarce.
Żaden moduł supervisora nie otwiera RTDE ani portu szeregowego. Kamerę
otwiera wyłącznie jawny tryb `--camera`. Detekcja różnicowa nie jest
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

`system_vision.py` czyta klatki we własnym wątku. Z klatek w oknie 0,3 s
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

Flaga `--simulate` jest obowiązkowa. Tryb sprzętowy nie jest jeszcze
zaimplementowany; program nie przełącza się do niego automatycznie.
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

Następny etap według [planu](system_supervisor_plan.md) to etap 3: adaptery
urządzeń (`SafeControl` + watchdog w `ControlLoop`, obsługa portów chwytaka
i magistrali serw poza pętlą), walidacja konfiguracji i nauka stanowisk
z panelu. Potem walidacja pojedynczej szklanki. Pozostają też: próba
`--camera` na prawdziwym stole, rozdzielenie kamery i panelu do osobnych
procesów, geometria obrotu, sprzętowe profile operacji, trwała zajętość
miejsc po restarcie oraz blokada drugiej instancji niezależna od portu. Nowe uruchomienie symulatora tworzy nowy fikcyjny świat;
dziennik nie służy do automatycznego wznawiania ruchu.
