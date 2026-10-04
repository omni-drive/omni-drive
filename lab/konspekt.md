# Programowanie Autonomicznych Robotów Mobilnych

## Robot OmniDrive z kołami omnikierunkowymi

wersja 1.0, 2026.10.04

---

## 1. Przygotowanie do zajęć

Przed zajęciami należy zapoznać się z budową robota OmniDrive. Robot porusza się na trzech kołach
omnikierunkowych rozstawionych co 120°, dzięki czemu może jechać w dowolnym kierunku i obracać się
w miejscu. Jest wyposażony w skaner laserowy (LiDAR), żyroskop i trzy czujniki ultradźwiękowe.

**W sali laboratoryjnej dostępny jest sprzęt komputerowy z potrzebnym oprogramowaniem.**

## 2. Wstęp

Na zajęciach wyznaczymy model ruchu robota i sprawdzimy, jak dokładnie robot zna swoje położenie.
Porównamy trzy sposoby jego wyznaczania: zliczanie obrotów kół (odometrię), dopasowanie skanu
LiDAR-u do mapy otoczenia oraz filtr Kalmana, który łączy pomiary z kilku czujników.

## 3. Zasady pracy z robotem

* **Robot jeździ wyłącznie wewnątrz pudła.**
* **Przed uruchomieniem ruchu sprawdź, czy w pudle nie ma przeszkód.**
* **Nie przenoś ani nie obracaj robota ręką, gdy jest w ruchu.**
* **Gdy robot nie przestaje jechać, zatrzymaj go poleceniem z notebooka.**
* **Jeśli zauważysz usterkę, powiadom prowadzącego.**

## 4. Uruchamianie robota

Włącz robota i poczekaj, aż zielona dioda zacznie migać.

Połącz komputer z siecią Wi-Fi robota, otwórz w przeglądarce adres podany przez prowadzącego
i zaloguj się hasłem do Jupytera.

Otwórz notebook `notebook.ipynb`.

**Ciąg dalszy w zawartości notebooka.**

## 5. Zakończenie zajęć

**Zatrzymaj robota i wyłącz jego zasilanie.**
