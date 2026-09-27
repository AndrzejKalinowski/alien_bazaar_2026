# Supervisor — pierwszy działający etap

Implementacja bazuje na `master` (`7c145d2`), na gałęzi
`feature/system-supervisor`. Rdzeń realizuje pełny cykl na symulowanych
urządzeniach. Żaden nowy moduł nie otwiera RTDE, portu szeregowego ani kamery.
Detekcja różnicowa nie jest importowana ani przenoszona do tej gałęzi.

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

Nadzorca ma jednego właściciela. Przyszły panel przekaże mu komendy kolejką;
nie wolno wywoływać jego metod równolegle z kilku wątków. `DeviceAdapter`
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

Następny etap według [planu](system_supervisor_plan.md): klasyczna wizja
z `master` oraz panel WWW sterujący symulacją. Następnie adaptery urządzeń,
nauka stanowisk i walidacja pojedynczej szklanki. Rozdzielenie procesów,
watchdog rzeczywistego robota, geometria obrotu, sprzętowe profile operacji,
trwała zajętość miejsc po restarcie i blokada drugiej instancji pozostają do
implementacji. Nowe uruchomienie symulatora tworzy nowy fikcyjny świat;
dziennik nie służy do automatycznego wznawiania ruchu.
