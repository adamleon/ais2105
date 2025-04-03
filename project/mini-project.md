# Mini-prosjekt

Dette mini-prosjektet er ment til å sette sammen alt dere har lært i labøving 1, 2 og 3. Dere skal lage et fullstendig ROS2-prosjekt, med flere pakker som samarbeider med dokumentert oppførsel, på lik linje med andre ROS2-prosjekter.

I dette prosjektet skal dere:
- Styre en Quanzer Qube med ROS2
- Ha en simulator ved siden av
- En launch fil som kan bytte mellom Qube og simulator.
- Visualisering i RViz
- Styre vinkel via en terminal og/eller GUI.

Dere skal levere inn koden, med dokumentasjon som viser hvordan systemene funker sammen, og kode som er dokumentert godt nok til å forstå hvordan den fungerer.
# Forberedelser
Lag en repository som dere kaller `qube` eller noe lignende. Den skal du legge inni `src` i en av workspace-ene dere planlegger å bruke (dere kan for eksempel lage en ny en for prosjektet). Inni her skal dere ha tre ROS2 pakker
- `qube_description` som inneholder den geometriske beskrivelsen av Quben
- `qube_driver` som inneholder kommunikasjonsgrensesnittet med den fysiske Quben
- `qube_bringup` som inneholder launch- og konfigurasjonsfiler og dokumentasjon.
- `qube_controller` som innholder en PID-kontroller som regulerer roboten.
Oppgavene under forteller dere hvordan dere skal lage de forskjellige pakkene.
# Oppgave 1: qube_description
Dere skal lage en pakke som dere kaller `qube_description` (bruk ament_python som byggetype). Dere skal lage to URDF-filer:
- `qube.macro.xacro:` som inneholder en makro som beskriver Quben
- `qube.urdf.xacro:` som inneholder en veldig enkel scene av Quben.
I `qube.macro.xacro` (heretter kalt makrofilen) skal dere beskrive Quben inni en macro som heter `qube`. Noen lignende dette:
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" >
	<xacro:macro name="qube">
		Her ligger beskrivelsen av Quben
	</xacro:macro>
</robot>
```
Quben skal være en svart boks, med en rød spinnende disk oppå. Disken skal ha en hvit viser som skal indikere vinklene til Quben. Bruk gjerne Lab 3 som inspirasjon. Hvis dere bruker Lab 3 som inspirasjon, fjern også link-en som heter `world` og joint-en som kobler `world` og `base_link`. Disse vil bli definert i andre filer.

Dere skal parameteriser verdiene til URDF-en, slik at lengde, bredde, osv. defineres som en variabel. Gjerne også legg til ekstra detaljer som bein og lignende.

I `qube.urdf.xacro` (som herved kalles scene-filen) lager dere en URDF som ser slik ut
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" >
	<xacro:include filename="$(find qube_description)/urdf/qube.macro.xacro" />
	Legg til en world-link
	Kall Qube makroen
	Og lag en joint som knytter base_link til world

</robot>
```
Alt dette er for å gjøre beskrivelsen gjenbrukbar. Det er meningen at makro-filen skal kunne brukes i andre URDF-beskrivelser som enten du eller andre beskriver. Eneste som trengs er en `xacro:include` og at du kaller makroen.

Scene-filen er nettopp det. En veldig enkel scene: En Qube som står i origo. Denne skal kunne brukes til å visualisere beskrivelsen i makroen.

Siste dere skal gjøre er å lage en launch-fil i en launch-mappe, som heter `view_qube.launch.py`

# Oppgave 2: qube_driver
Neste dere skal gjøre er å koble opp Quanser Quben. Det gjøres ved at dere skal bruke ROS2 Control til å kontrollere roboten. ROS2 Control er et sett med noder som kobler sammen styresystemer i ROS2 og kommunikasjonen opp mot roboten.

Det første du må gjøre er å sikre at du har ROS2 Control installert
```
sudo apt install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers
```

Som dere kan se under [arkitekturen](https://control.ros.org/rolling/doc/getting_started/getting_started.html#architecture) kan dere se at ROS2 Control deler seg inn i to:
- En controller-del hvor alle styresystemene opererer. Dette kan være alt fra styring av IO til PID-controllere, til styring via inverskinematikk.
- En hardware-del hvor all kommunikasjonen opp mot maskinvaren foregår. Dette kan være EtherCAT-kommunikasjon opp mot en robot, simulert kommunikasjon mot en simulator, eller som i vårt tilfelle, seriellkommunikasjon mot en Arduino (eller Teensy).
Hardware-delen er veldig spesifikk, og er derfor allerede laget i en egen pakke som heter `qube_driver`. Denne pakken kan du laste ned [her](https://github.com/adamleon/qube_driver) (eller aller helst bruke `git clone`). Dere trenger ikke å gjøre noen spesifikke endringer på denne akkurat nå.

Bygger du workspace nå med `qube_driver` lastet ned, skal du ikke ha noen problemer. Det vil komme noen warnings, skyld på Lars Ivar. De er bare å ignorere.

# Oppgave 3: qube_bringup

Lag en ny pakke som heter `qube_bringup` bruk ament_python som byggetype. Denne pakken skal inneholde launch- og konfigurasjonsfiler for flette sammen Qube-systemet.

Dere skal lage en ny mappe som heter urdf, hvor vi skal lage en ny URDF scenefil (kall filen `controlled_qube.urdf.xacro` eller noe annet). Denne skal være lik `qube.urdf.xacro` i innhold, men skal ha mer. Den skal inkludere filen `qube_driver.ros2_control.xacro` som ligger under `qube_driver/ros2_control` (bruk `xacro:include`).

Ser dere inni ROS2-control-filen, ser dere at en ny makro defineres, som heter `qube_driver_ros2_control`. Den har også en `params="name baud_rate..."` som definerer parametere man kan sette når man kaller makroen. 

I "controlled_urdf"-filen, definerer dere tre macro argumenter (`xacro:arg`): baud_rate, device og simulation. Etter dere kaller qube-makroen, skal dere kalle makroen `qube_driver_ros2_control`, hvor dere setter parameterne til makroen (`name="qube_driver", prefix="", baud_rate=$(arg baud_rate)` osv.).

TIlslutt skal dere lage en launch fil, som heter `bringup.launch.py`. Den skal starte opp `qube_driver.launch.py` fra qube_driver-pakken, rviz og robot state controller (hvor du laster inn controlled_qube.urdf.xacro). 

Det som skjer når dere kjører denne filen er at dere starter opp robot state publisher, som holder orden på robotens tilstand. Dere starter også opp en ROS2 control controller manager som håndterer samhandlingen mellom controller-delen og hardware-delen i qube_driver. Videre starter opp joint state broadcaster, som leser verdiene fra hardware-delen og publiserer de på `/joint_states`. Kjører dere `ros2 topic echo /joint_states` vil dere se at posisjon, og hastighet fra quben blir publisert. Tilslutt starter velocity controller opp. Det er en type kontroller som lastes inn i controller-delen og sender hastighetskommandoer tilbake til hardware-delen.
# Oppgave 4: qube_controller
Den siste delen er å styre quben. Dere skal lage en node som abonnerer til `/joint_states` og henter ut posisjon og hastighet til quben. Videre skal dere lage en PID-controller som regulerer og gir ut et hastighets-pådrag. Det skal dere publisere til `/velocity_controller/commands`. Merk at meldingstypen er `Float64MultiArray` og krever derfor en spesiell måte å populeres på. Les på dokumentasjonen for å se hvordan.

# Koble opp Quben
For å koble til Quben, så må dere finne ut hvilken `device` og `baud_rate` som dere må sette opp. Baud rate er 115200 med mindre noe annet er sagt. Device er navnet på USB-porten. For å finne navnet, kan du kjøre
```
ls /dev/tty*
```
Den vil liste opp alle kommunikasjonene som er tilgjengelig på PCen. Du vil finne en som heter /dev/ttyACMX, hvor X er et tall, vanligvis 0. Dette er navnet `/dev/ttyACMX`.

Hvis dere starter launch-filen og dere for en nserialIOException, med en "Permission Denied", så er det fordi du ikke har gitt lese/skriverettigheter til USB-porten. For å gi tilgang kjører dere kommandoen
```
sudo chmod 666 /dev/ttyACMX
```
`chmod` står for CHange MODe, for å endre rettigheter. 666 står for hvilke rettigheter gis til hvem. Det første tallet er filens eier, andre tallet er gruppen filen er i, og siste tallet er alle andre. Tallet er et binært tall som setter bit for read, write og execute (rwx), 6=110 som gir rettigheter til lesing og skriving.

Merk også at hvis du får feilmelding om "motor_joint not found", så er det fordi qube_driver antar at URDF-en din har et ledd som heter "motor_joint", altså det roterende leddet i Quben. Hvis du har kalt det "rotary_joint", "disk_joint" eller noe annet, så får du feil.

# Oppgave 5: Selvevaluering
Det siste dere skal gjøre er å evaluere deres eget arbeid. Dette gjør dere individuelt, og leveres inn på blackboard sammen med en link til repo-et dere har laget.

Dere skal svare i prosent hva dere tenker om arbeidet dere har gjort i prosjektet

## qube_description
- Hvor godt dokumentert er filene i pakken?
- Hvor godt dokumentert er selve pakken?
- Hvor lik er URDF-beskrivelsen en ekte Qube?
- Hvor godt funker launch-filen til å visualisere quben?
## qube_bringup
- Hvor godt funker launch-filen til å starte opp alle elementene?
- Hvor enkelt kan man endre på hardware-paramererne som baud_rate, device og simulation? (Det å endre parametrene når man kjører launch-filen er enkelt. Det å måtte endre en hardkodet verdi en tilfeldig plass inni launch-filen og måtte bygge den etterpå er ikke enkelt.)
- Hvor godt dokumentert er pakken?
## qube_controller
- Hvor godt regulerer regulatoren?
- Hvor enkelt er det å endre på parametere?
- Hvor godt dokumentert er pakken?
- Hvor enkelt er det å endre på referansevinkelen?
