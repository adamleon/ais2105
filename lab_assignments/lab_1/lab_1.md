# Lab 1 - Topics og Parameters
I denne laboppgaven skal dere lage en PID-kontroller for et ledd til en robot. Dere skal også lage en simulator av leddet for å teste styresystemet.

Dette er en oppgave for to personer, hvor dere gjør to forskjellige oppgaver som til sammen blir en helhet. Den ene lager kontrolleren (oppgaver merket med A) og den andre lager simulatoren (oppgaver merket med B). På grunn av hvordan ROS2 er oppbygd skal PCene deres kunne kommunisere med hverandre hvis dere er på samme nettverk. Merk at dere burde jobbe for at begge blir ferdige med oppgave 1 før dere begynner på oppgave 2, osv. Fordi det er enklere å sjekke at nodene fungerer når de blir testet sammen.

Det er lurt om dere går gjennom veiledninger fra https://docs.ros.org/en/jazzy/Tutorials.html. Hvis dere har gått gjennom Beginner CLI og Beginner Client, vil denne laben være mye enklere. Du kan selv velge om du vil programmere i C++ eller Python. Python er muligens enklest i starten, men C++ er muligens enklest i lengden. Alt innen ROS fungerer like bra uansett om du velger C++ eller Python, så du kan bytte frem og tilbake slik du vil.

Det er kan også være nyttig å jobbe sammen med et repository. Det er flere ganger hvor du trenger filer/kode fra makkeren din, så da er det enklest å gjøre det via repository.
# Oppgave 1
## Del A
Det første du skal gjøre er å lage en ny [ROS2 pakke](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html). Kall den `pid_controller` eller noe annet passende. Du velger selv om du vil skrive den i C++ eller Python (ament_cmake eller ament_python). Koden beskrevet i oppgaven er i Python.

Lag så en ny fil i pakken hvor du legger koden (i `pid_controller` hvis du velger Python, eller `src` for C++). Kall den `pid_controller_node.py` (eller `.cpp`).

Lag en ny klasse som heter `pidController` som har fem variabler `p`, `i`, `d`, `reference` og `voltage`, og en `update`-funksjon som oppdaterer `voltage` basert på de andre variablene. Lag også flere variabler og funksjoner for å løse dette.

Når klassen fungerer, endre koden så du inkluderer en [ROS2 node](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html) (for eksempel ved å lage klassen `def PIDControllerNode(Node):`) som har en instans av `pidController` der du initialiserer den med litt tilfeldige PID-parametere. Den skal også ha en *publisher* som heter `publish_voltage` som publiserer en `Float64` melding med `voltage`-verdien, og en *subscriber* som heter `measured_angle` og en callback-funksjon kalt `measurement_listener` som mottar en `Float64`-melding, bruker den verdien til å kalle `update` til PID-kontrolleren og kaller på `publish_voltage` for å publisere det oppdaterte pådraget.

Gjør de nødvendige endringene i pakken for å bygge (for eksempel å endre `setup.py` til å legge til den nye noden `pid_controller_node = pid_controller_node.main`). Test ut noden ved å kjøre
```
ros2 run pid_controller pid_controller_node
```
For å se verdiene som blir publisert fra topic-en `voltage` kan du kjøre
```
ros2 topic echo /voltage
```
Du kan også kjøre `rqt` for å få et visualiseringsverktøy. Velg `Plugins>Visualization>Plot` for å få opp en graf. Under Topic kan du velge /voltage/data (altså "topic-navn"/"navn på variabel i meldingen")

Du kan teste om noden fungerer ved å publisere en melding fra kommandolinjen
`ros2 topic pub -1 /measured_angle /std_msgs/Float64 "{data: 5.0}"`
(Oversatt: Publiser 1 melding til `/measured_angle` av typen `/std_msgs/Float64` med beskjeden `{data: 5.0}` som er skrevet i YAML)

For å sjekke at dette fungerer med del B, kan dere koble PC-ene sammen med Ethernett-kabel (altså koble de til samme vegg/bord). Hvis person A kjører
`ros2 run pid_controller pid_controller_node`
og person B kjører
`ros2 run joint_simulator joint_simulator_node`
Det skal nå være slik at PC-ene kommuniserer med hverandre, og `effort`-meldingene sendes fra ene og mottas av andre, det samme med `measured_value`.
## Del B
Det første du skal gjøre er å lage en ny [ROS2 pakke](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html). Kall den `joint_simulator` eller noe annet passende. Du velger selv om du vil skrive den i C++ eller Python (ament_cmake eller ament_python). Koden beskrevet i oppgaven er i Python.

Lag så en ny fil i pakken hvor du legger koden (i `joint_simulator` hvis du velger python, eller `src` for C++). Kall den `joint_simulator_node.py` (eller `.cpp`).

Lag en ny klasse som heter `jointSimulator` som har fire variabler `angle`, `angular_velocity`, `voltage` og `noise`, og en `update`-funksjon som oppdaterer `angle` basert på formelen
$$
\frac{\theta(s)}{V(s)} = \frac{K}{s(Ts+1)}
$$
Der $\theta$ er vinkel, $V$ er spenning og $K=230 \text{rad}$ og $T=0.15\frac{\text{rad}}{\text{s}}$ er konstanter. Lag også flere variabler for å løse dette. Enn så lenge kan du sette `noise=0`.

Når klassen fungerer, endre koden så du inkluderer en [ROS2 node](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html) (for eksempel ved å lage klassen `def JointSimulatorNode(Node):`) som har en instans av `jointSimulator`. Den skal også ha en *publisher* som heter `publish_angle` som publiserer en `Float64` melding med `angle`-verdien, og en *subscriber* som heter `input_voltage` og en callback-funksjon kalt `voltage_listener` som mottar en `Float64`-melding, som oppdaterer `voltage`. Noden skal også ha en `wall_timer` (slik som i tutorialen) som kaller `update` til simulatoren og regner den nye vinklen og kaller på `publish_angle` for å publisere det oppdaterte vinkelen.

Gjør de nødvendige endringene i pakken for å bygge (for eksempel å endre `setup.py` til å legge til den nye noden `joint_simulator_node = joint_simulator_node.main`). Test ut noden ved å kjøre
```
ros2 run joint_simulator joint_simulator
```
For å se verdiene som blir publisert fra topic-en `voltage` kan du kjøre
```
ros2 topic echo /angle
```
Du kan også kjøre `rqt` for å få et visualiseringsverktøy. Velg `Plugins>Visualization>Plot` for å få opp en graf. Under Topic kan du velge /angle/data (altså "topic-navn"/"navn på variabel i meldingen")

Du kan teste om noden fungerer ved å publisere en melding fra kommandolinjen
`ros2 topic pub -1 /input_voltage /std_msgs/Float64 "{data: 5.0}"`
(Oversatt: Publiser 1 melding til `/input_voltage` av typen `/std_msgs/Float64` med beskjeden `{data: 5.0}` som er skrevet i YAML).

## Del A og B
For å sjekke at dette fungerer mellom person A og person B, kan dere koble PC-ene sammen med Ethernett-kabel (altså koble de til samme vegg/bord). Hvis person A kjører
`ros2 run pid_controller pid_controller_node`
og person B kjører
`ros2 run joint_simulator joint_simulator_node`
Det skal nå være slik at PC-ene kommuniserer med hverandre, og `voltage`-meldingene sendes fra ene og mottas av andre, det samme med `measured_angle`. Hvis du nå endrer på `reference` i PID-noden, skal du se at `measured_angle` også endrer seg.
# Oppgave 2
## Del A
Det siste du skal gjøre er å parameterisere PID parameterene. Dette gjør du ved å endre på PID-kontrollernoden. Eksempler er vist [her](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)
Setter man `self.declare_parameter('p', 1.0)` vil du sette `p=1` på parameterserveren. Du kan rett etterpå i koden skrive
`self.p = self.get_parameter('p').get_parameter_value().float64_value` for å hente parameteren fra parameterserveren og sette variabelen din til den. Gjør så det samme for I og D

Du kjører noden kan du kjøre 
`ros2 run pid_controller pid_controller_node --ros-args -p p:=10.0 -p d:=1.0` og se at parameterene blir satt når du kjører.

Du kan også se parameterene ved å kjøre
`ros2 param list` og `ros2 param get /pid_controller_node p`

Hvis du prøver å endre parametrene med
`ros2 param set /pid_controller_node p 5.0` vil du legge merke til at parameteren endrer seg på parameterserveren, men ikke i noden. Dette er fordi noden ikke har blitt fortalt at parameteren har blitt endret. Dette kan du endre på ved å kalle funksjonen 
`self.add_on_set_parameters_callback(self.parameter_callback)`
Etter at alle parameterene er deklarert. Deretter definerer du funksjonen `parameter_callback`
```python
def parameter_callback(self, params):
        """Callback to handle parameter updates."""
        for param in params:
            if param.name == 'p':
                if (param.value >= 0.0):
                    self.p = param.value
                    self.get_logger().info(f' p was set: {self.p}')
        # same for i and d

```
Merk at funksjonen kun oppdaterer P. Gjør endringer for å oppdatere I og D også.

Når du nå kaller `ros2 param set...` så vil også parameterene i noden endre seg.
## Del B
Det siste du skal gjøre er å parameterisere simulatorverdiene. Dette gjør du ved å endre på simulatornoden. Eksempler er vist [her](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)
Setter man `self.declare_parameter('noise', 1.0)` vil du sette `noise=1` på parameterserveren. Du kan rett etterpå i koden skrive
`self.noise = self.get_parameter('noise').get_parameter_value().float64_value` for å hente parameteren fra parameterserveren og sette variabelen din til den. Gjør så det samme for K og T.

Du kjører noden kan du kjøre 
`ros2 run joint_simulator joint_simulator_node --ros-args -p K:=10.0 -p T:=1.0` og se at parameterene blir satt når du kjører.

Du kan også se parameterene ved å kjøre
`ros2 param list` og `ros2 param get /joint_simulator_node K`

Hvis du prøver å endre parametrene med
`ros2 param set /joint_simulator_node K 5.0` vil du legge merke til at parameteren endrer seg på parameterserveren, men ikke i noden. Dette er fordi noden ikke har blitt fortalt at parameteren har blitt endret av parameterserveren. Dette kan du endre på ved å kalle funksjonen 
`self.add_on_set_parameters_callback(self.parameter_callback)`
etter at alle parameterene er deklarert. Deretter definerer du funksjonen `parameter_callback`
```python
def parameter_callback(self, params):
        """Callback to handle parameter updates."""
        for param in params:
            if param.name == 'noise':
                if (param.value >= 0.0):
                    self.noise = param.value
                    self.get_logger().info(f' noise was set: {self.noise}')
        # same for K and T

```
Merk at funksjonen kun oppdaterer noise. Gjør endringer for å oppdatere K og T også.

Når du nå kaller `ros2 param set...` så vil også parameterene i noden endre seg.
## Del A og B
For å sjekke at dette fungerer mellom person A og person B, kan dere koble PC-ene sammen igjen. Hvis person A kjører
`ros2 run pid_controller pid_controller_node`
og person B kjører
`ros2 run joint_simulator joint_simulator_node`
Dere vil nå se at både `measured_angle` og `voltage` endrer karakteristikk hvis dere endrer på parameterne.
