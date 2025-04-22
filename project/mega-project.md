På overordnet nivå skal dere programmere en UR-robot til å ta bilde av noen fargede kuber som skal kunne plasseres tilfeldig på et bord, og skal kunne peke på kubene i rekkefølge.

Dere vil bli målt på:
**ROS2: 6 poeng**
- Hvor godt er nodene strukturert i forhold til hverandre
- Hvor godt er nodene implementert (riktig bruk av launch, config, osv.)
- Hvor konfigurerbart er det (bruk av parameter, argumenter i launch, o.l.)

**Kamera og robot: 10 poeng**
- Hvordan er kamera-pipeline satt sammen?
- Hvor robust er kamera til å detektere kubene (forskjellige posisjoner, høyder, lysforhold, osv.)
- Hvordan beveger roboten seg (hvor presist, hvordan peker den, korrigerer den underveis, o.l.)

**Skriving: 10 poeng**
- Hvor godt skrevet/strukturert er rapporten
- Hvor godt vurderes metoder, implementering o.l. opp mot hverandre
- Hvor godt er teoriforståelse i faget

**Samarbeid: 6 poeng**
- Hvor godt samarbeider gruppen, og hvordan var arbeidsfordelingen
- Hvilke verktøy er brukt innad i gruppen (git, overleaf, discord, osv.)

**Ekstra: 8 poeng**
- Det tildeles ekstra poeng for det som gjøres utover kravene til oppgaven.
- Det blir sett etter om dere gir robotsystemet tilleggsfunksjoner som for eksempel programmerte sikkerhetssoner, deteksjon av feil tilstand (for eksempel at en farge mangler), plukke opp og sortere kuber, bruke noe annet enn kuber, oppkobling mot PLS eller andre sensorer eller motorer, bruk av kraftstyring, bygge noe rundt, osv.
- Dere trenger ikke å gjøre alt, og det vil vurderes ut ifra kvalitet. Så en godt implementert ekstrafunksjon er bedre enn ti dårlig implementerte
- Med i denne vurderingen vil også anvendelse og relevans bli tatt i betraktning. Hvis dere velger å forme løsningen rundt en industriell problemstilling (del av en produksjonsprosess, løse et problem for en bedrift, e.l.), vil det også telle.
- Det gjør ingenting om dere må gjøre forenklinger eller gjør en dårligere jobb enn en reell industriell løsning, så lenge det rettferdiggjøres i rapporten.

Hver UR-robot er utstyrt med et Surface-brett som er ferdig installert med Linux og ROS2, samt pakker for å koble seg til roboten. Kjører dere et par launch-filer skal dere kunne kommunisere med nettbrettet. Dere kan da lage og kjøre noder på egne PCer for å styre roboten.

Dere velger fritt med antall noder og strukturen på disse, men bruk gjerne kunnskapen dere har opparbeidet dere i løpet av semesteret.
- Dere skal kunne flytte roboten til en hjemposisjon, som er en nøytral posisjon (for eksempel 90° knekk på albue). Dere skal kunne kjøre en node, en launch-fil eller en kommando (via GUI eller terminal) som flytter roboten dit.
- Roboten skal kunne gjøre en serie med oppgaver når den starter:
	1. Flytte seg til å kunne ta et oversiktsbilde over arbeidsbordet.
	2. Detektere tre kuber i forskjellige farger fra dette bildet (foreløpig rød, gul og blå, men fargen kan variere).
	3. Roboten skal så bevege seg nært (typ 10 cm, men det trengs ikke være nøyaktig) til først den røde, så den gule så den blå. Roboten kan bevege seg via et mellompunkt mellom de tre.
	4. Hvis roboten ikke detekterer en farge, skal den prøve å lete etter den. Det kan være så enkelt som å flytte til en ny posisjon og ta bilde, men kan også være at den tar en serie med bilder fra flere posisjoner.
	5. Hvis den fortsatt ikke finner kuben skal den stoppe og gi en varsel.
- Dere kan som sagt få ekstra poeng ved å utvide disse kriteriene, og også å fjerne noen for å erstatte det med noe annet. Dette må i så fall rettferdiggjøres, og det er lurt om dere spør først.

# Hvordan koble seg til en robot
Hver robot har som sagt et Surface-brett med Linux som er installert og konfigurert til å koble til den roboten.

Hver robot har et pokemon-navn og det tilsvarende brettet vil ha samme navn. UR3/UR3e-roboter har navn etter basis-pokemon, UR5/UR5e etter 1.-stadie pokemon og UR10/UR10e er 2.-stadie pokemon.

De har fast IP-adresser (eller skal ha, hvis noen tukler med IP-adressene). Addresses er 143.25.150.X for robotkontrolleren og 143.25.151.X for nettbrettet, hvor X er nummeret på pokemonen i Pokedexen. Subnet Mask skal være 255.255.252.0, default gateway er 143.25.151.0 og DNS er 0.0.0.0. Hvis dere skal koble dere på nettverket med egen PC velger dere 143.25.150.(gruppenummer) eller 143.25.151.(gruppenummer) med samme subnet, gateway og DNS.

For å koble deg på roboten, starter du opp roboten og kobler Ethernet-kabelen til sokkelen i et arbeidsbord. Deretter kobler du til nettbrettet til samme arbeidsbord med en kabel.

Start opp nettbrettet. Det er mulig det først starter i Windows. 
- Gå på startmenyen og og trykk Restart mens du holder inne shift.
	- Hvis det ikke funker: Gå da på Start->Settings->Security and Updates->Recovery. Under Advanced Startup trykk "Restart Now"
- Inne i Advanced Startup-modus, trykker du "Use a Device" og velg Ubuntu.
- Når Ubuntu starter, velger du Ubuntu igjen
- Logg deg inn med brukeren med passord "iir_robot".
## Koble opp kontrolleren
Bruker du en e-serie-robot kan du kjøre "headless mode", altså at ROS2 laster over et script som roboten kjører uten å gjøre konfigurasjoner. Sett roboten i "remote mode" (øverst i høyre hjørne ved siden av menyen). Deretter kjører du kommandoen
```
ros2 launch ros2 launch ur_robot_driver ur_control.launch.py ur_type:=urX ​robot_ip:=yyy.yyy.yyy.yyy use_mock_hardware:=false
initial_joint_controller:=joint_trajectory_controller headless_mode:=true
```
Hvor urX er typen til roboten og robot_ip er IP-adressen som nevnt tidligere over. Sjekk gjerne på kontrolleren at adressen er satt riktig. Du kan også kjøre:
```
ping 143.25.150.X
```
for å se om du har kontakt.

For en CB3-serie-robot må dere ha en URCap som heter External Control. Hvis ikke det er et program allerede som heter ROS2, så kan dere enkelt lage et nytt et og legge inn External Control som eneste funksjonsblokken. Sørg for at IP adressen er den samme som nettbrettet, og trykk Play. Da venter den på kompandoer fra brettet.

På nettbrettet kjører dere så komandoen
```
ros2 launch ros2 launch ur_robot_driver ur_control.launch.py ur_type:=urX robot_ip:=yyy.yyy.yyy.yyy use_mock_hardware:=false
initial_joint_controller:=joint_trajectory_controller
```
Hvor urX er typen til roboten og robot_ip er IP-adressen som nevnt tidligere over. Sjekk gjerne på kontrolleren at adressen er satt riktig. Du kan også kjøre:
```
ping 143.25.150.X
```
for å se om du har kontakt.

## MoveIt
Dere kan også starte MoveIt for å styre roboten med. I en ny terminal, kjør kommandoen 
```
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=urX launch_rviz:=true
```
I så fall kan dere velge p sette launch_rviz:=false *i den første* kommandoen så bare et vindu er oppe. Roboten skal nå kunne styres via RViz. For å programmere en node som styrer via MoveIt fra egen PC, kan dere se på ressurser [her](https://moveit.picknik.ai/main/doc/tutorials/tutorials.html).
## Problemer?
Hvis dere ikke får koblet opp riktig:
- Sjekk at alle kablene er plugget i riktig
- Pass på at alle IP-adresser er satt riktig
- Pass på at WiFi og lignende er skrudd av. Det kan hende PCen velger å kommunisere via den.
- Sjekk på UR sin dokumentasjon om hvordan roboten skal kobles opp [her](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/usage/toc.html). Her vil dere også finne flere ressurser for å sette opp et godt prosjekt.

# Rapport
Rapporten til slutt skal ha følgende oppbygning:
- Introduksjon: Beskriv problemstillingen dere prøver å løse. Gjerne med en konkret beskrivelse av hvilke kriterier dere har satt (roboten skal peke på trek kuber med å stille seg *minst* 10 cm unna, e.l.)
- Teori: Skriv det teoretiske grunnlaget for de metodene og verktøyene dere har brukt i oppgaven. Ting som er antatt "vanlig kunnskap" blant studenter (for eksempel ROS2) kan sløyfes, med mindre dere bruker noe spesifikt videre i oppgaven (for eksempel ROS2 Actions eller biblioteker).
- System: En beskrivelse av hele systemet og alle undersystem (for eksempel kamera/deteksjonssystem og beslutningssystem). Tegn gjerne diagram som viser sammenhengen. Robotkontrolleren, UR-driver og annet som allerede er satt opp skal nevnes, men trengs ikke å beskrives nøye.
- Resultat: Beskriv tester dere har gjort når dere tester hele systemet (hvor mange kuber, hvor mange tester, hvor nært pekte roboten, oppdaget den feil osv.) Og presenter resultatene fra testene.
- Diskusjon: Hvor godt gjorde roboten det i forhold til målene dere satt i introduksjonen? Hvorfor funket det/funket det ikke.
- Gruppedynamikk: Her skriver dere hvem som gjorde hva, hvordan dere arbeidet, om dere brukte noen spesifikke verktøy som git, Teams e.l. Til slutt skriver dere individuelt en refleksjon om gruppedynamikken. Den individuelle delen skal være helt individuell og ingen andre på gruppen skal se den. Dette kan leveres som et separat dokument hvis det passer bedre.
- Konklusjon: Summer resultatene og introduksjonen.
Tenk over poengfordelingen presentert i starten av oppgaven. Vær sikker på at dere svarer godt på de kriteriene i rapporten for å sikre at dere får uttelling.
