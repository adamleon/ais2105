I denne laboppgaven skal dere lage en ROS2 service, URDF- og launch-filer for leddet dere lagde i Lab 1. Gjør derfor Lab 1 først. Det anbefales også at dere gjør tutorialen for [ROS2 Services](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html).

Dette er en oppgave for to personer, hvor dere gjør to forskjellige oppgaver som til sammen blir en helhet. Den ene lager URDF og service-server (oppgaver merket med A) og den andre lager launch og service-klient (oppgaver merket med B). På grunn av hvordan ROS2 er oppbygd skal PCene deres kunne kommunisere med hverandre hvis dere er på samme nettverk. Merk at dere burde jobbe for at begge blir ferdige med oppgave 1 før dere begynner på oppgave 2, osv. Fordi det er enklere å sjekke at nodene fungerer når de blir testet sammen.

Det er kan også være nyttig å jobbe sammen med et repository. Det er flere ganger hvor du trenger filer/kode fra makkeren din, så da er det enklest å gjøre det via repository.

# Oppgave 1
## Del A
Oppgaven din er å lage en egendefinert service-melding. Det første du skal gjøre er å lage en ny [ROS2 pakke](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html) (MED `--build_type ament_cmake`!). Kall den `pid_controller_msgs`. Følg tutorialen i linken, men kun lag en egendefinert `srv` ikke `msg`. 

Lag en mappe som kalles `srv` og lag en fil som heter `SetReference.srv`. Kopier dette inn i filen
```
Float64 request
--
Bool success
```
Når du nå kompilerer pakken, vil du få en ny service-melding som mottar en Float64 som heter request. Det er referanseverdien som service-klienten ber om skal endres til. Service-serveren prosesserer verdien og returnerer en bool success som er enten true hvis referanseverdien er satt riktig, eller false hvis den ikke er satt riktig. Husk å sett opp `CMakeLists.txt` og `package.xml` riktig sånn at service-meldingen blir kompilert.

Pass på å kopiere denne pakken over til partneren din (eller oppdater repository så dere får samme pakken). Partneren din trenger å kompilere denne pakken på sin egen PC for å kunne bygge sin kode.

## Del B
Det første du skal gjøre er å utvide `pid_controller_node` noden din til å inkludere en service-server. Følg tutorialen [her](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html) og se på hvordan du lager en *server*, med noen endringer.

I tutorialen lager du en service server som lytter til `add_two_ints` og som bruker service-meldingen `AddTwoInts`. Du skal lage en server som lytter til `set_reference` og som bruker service-meldingen `SetReference`. `SetReference`-meldingen lages av partneren din i Del A, så kopier pakken eller last ned pakken fra repository-en deres slik at du enten legger den i din egen workspace, eller i en workspace som du "source"-er.

Du skal skrive om callback-funksjonen slik at når en service-melding blir sendt til `pid_controller_node` skal du sjekke om verdien er gyldig (et tall mellom $-\pi$ og $\pi$). Hvis det, oppdaterer du referansen til PID-kontrolleren og returnerer true, hvis ikke, returnerer du false.

## Del A og B
Når dere begge er ferdige skal dere starte opp PID-kontrolleren og simulatoren.

Dere kan nå kjøre `rqt` og finne "Service Caller" i Plugins. Bruk denne til å finne servicen "/pid_controller_node/set_reference", og velg en referanseverdi. Når du trykker "Call" vil forhåpentligvis referanseverdien endres.

# Lab 2 - Service og Launch
Dere kan også finne "Plot" i Plugins, og velge "/joint_simulator_node/measured_angle/data" og se vinkelen endre seg når du endrer på referanseverdien.

## Oppgave 2
I denne oppgaven skal dere se nærmere på bruken av launch-filer for å starte flere noder og URDF-filer for å visualisere "roboten".
## Del A
Du skal lage en [service-klient](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html), som du kaller `reference_input_node`, hvor den sender en melding til `set_reference` når du taster inn et nytt tall i terminalen. Eksempel på hvordan du kan taste inn tall er
``` python
userinput = int(raw_input("Sett inn ny referanse!\n"))
print("Den nye referansen er %d"%userinput)
```
### Del B
Du skal lage en [launch-fil](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html). Plasser `launch`-mappen inni `pid_controller`-mappen. Det som er beskrevet i tutorialen, starter opp tre noder: to `turtle_sim`-noder med forskjellige navn og en `mimic`-node. Hvis du har fulgt turtlesim-tutorialene så gir dette mening.

Din oppgave er å istedenfor starte en `pid_controller`-node, en `joint_simulator`-node og `reference_input_node`. 
### Del A og B
Dere skal nå kunne kjøre
```
ros2 launch pid_controller launch.py
```
og kan nå styre simulatoren og PID-kontrolleren ved hjelp av service-klienten.

## Oppgave 3
### Del A
Det siste du skal gjøre er å parametrisere launch-filen. Følger du [denne lenken](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html#parameters) finner du et punkt hvor du ser hvordan man kan definere launch-parametre, og hvordan man kan bruke dette til å sette parametre til nodene i filen. Eksempelvis 
``` python
Node(
    package='pid_controller',
    executable='pid_controller_node',
    name='pid_controller',
    parameters=[{
        'kp': LaunchConfiguration('kp'),
        'ki': LaunchConfiguration('ki'),
        'kd': LaunchConfiguration('kd'),
         }]
      ),
```

Sett opp slik at man kan sette PID-parametrene når man kjører launch-filen. Altså
```
ros2 launch pid_controller <navn på launch>.py kp:=10 ki:=0 kd:=5
```
### Del B
I samme mappen som `launch`-mappen lager du en `config`-mappe. Lag en fil som heter `parameters.yaml`. Du skal skrive filen slik at den inneholder parametere for `joint_simulator_node`. Du kan følge tutorialen [her](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html#loading-parameters-from-yaml-file)for å finne ut hvordan dette gjøres. Endre launch-filen (fra Del A) slik at du laster inn config-filen og setter de når du starter `joint_simulator_node`. Noe lignende
``` python
   config = os.path.join(
      get_package_share_directory('pid_controller'),
      'config',
      'parameters.yaml'
      )

   return LaunchDescription([
      Node(
         package='joint_simulator',
         executable='joint_simulator_node',
         name='joint_simulator',
         parameters=[config]
      )
   ])
```
### Del A og B
Når dere er ferdige, kan dere kjøre
```
ros2 launch pid_controller <navn på launch>.py kp:=0 kd:=5 ki:=1 
```
Og samtidig blir parameterne til simulatoren satt.
