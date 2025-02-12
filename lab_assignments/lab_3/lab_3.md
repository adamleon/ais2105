I denne laboppgaven skal dere konfigurere opp en URDF for å visualisere simulatoren deres. I tillegg skal dere lage en GUI så dere kan styre visualiseringen (ikke simulatoren... enda).

Dette er en oppgave for to personer, hvor dere gjør to forskjellige oppgaver som til sammen blir en helhet. Den ene lager en konfigurasjons fil for å starte opp mange prosesser (oppgaver merket med A) og den andre lager URDF (oppgaver merket med B). På grunn av hvordan ROS2 er oppbygd skal PCene deres kunne kommunisere med hverandre hvis dere er på samme nettverk. Merk at dere burde jobbe for at begge blir ferdige med oppgave 1 før dere begynner på oppgave 2, osv. Fordi det er enklere å sjekke at nodene fungerer når de blir testet sammen.

# Oppgave 1
## Del A
Det første du skal gjøre er å lage en ROS2 pakke, slik som du gjorde forrige labøving. Kall den `joint_description` eller noe lignende. Det kan være lurt å se gjennom tutorialen om [å sette opp URDF](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html) mest for å skjønne hva links og joints er.

Pakker er ikke bare hvor man legger kode, men man kan sette opp konfigurasjoner, parametere, meldinger, og mye mer. I første omgang skal du lage en launch-fil.

Launch-filen skal hete `view_model.launch.py` og skal starte opp en RViz-node (package `rviz2`, executable `rviz2`). Du kan også teste om RViz funker med å kjøre
```
ros2 run rviz2 rviz2
```

I tillegg skal du starte opp (i launch-filen) en "Robot state publisher"-node (package `robot_state_punlisher`, executable `robot_state_publisher`). Den er noden som oppdaterer tilstanden til roboten (blant annet hvordan leddene er koblet til hverandre).

Du må også legge til en URDF-beskrivelse til denne noden for at den skal fungere. Det gjør du ved å laste inn en URDF-fil og setter parameteren `robot_description` til innholdet av filen (`<robot>...</robot>`).
```python
robot_description_content = '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"  name="robot"><link name="world"/></robot>'

node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}] # adds a URDF to the robot description
    )
```
Over er et eksempel på hvordan det kan gjøres. Du ser at "content" i dette tilfellet er bare en robot med en link som heter world. Del B fokuserer på å generere en URDF som faktisk gir mening.

Når du kjører launch-filen, vil du se at RViz starter opp. I den venstre sidemenyen vil du finne en "Add"-knapp. Trykk den og velg "RobotModel". Finn RobotModel på sidemenyen og utvid. Der ser du mange konfigurasjoner du kan sette. Blant annet "Robot Description". Velg "robot_description". Nå lytter RViz på "robot_description"-topicen fra robot state publisher.

Du vil kanskje se noen feilmeldinger. Det er fordi RViz ikke klarer å finne transformasjonen mellom "world" (som er linken i URDF-en og robot_description), og "map" som er default-linken i RViz (som da ikke eksisterer i URDF-en). Velg "world" på Fixed Frame under Global Options.

## Del B
Du skal lage en URDF-fil som skal brukes i Del A. Lag en ny pakke (eller aller helst del repository med Person A, fordi dere lager samme pakken). Kall den `joint_description`, lag en mappe inni som heter `urdf`.

Bruk tutorialen for [å sette opp URDF](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html) til å lage en enkel modell av simulatoren vår (kall den `joint_model.urdf`). Merk at *tutorialen* antar at du jobber inni `urdf_tutorial`-pakken. Hvis du skal for eksempel kjøre `ros2 launch urdf_tutorial display.launch.py model:=urdf/<din modell>` så leter den etter `urdf_tutorial\urdf\<din modell>`. Det er hardkodet inn, så hvis du vil visualisere filen `joint_model`, må du kopiere den inn i `urdf_tutorial\urdf`. Hvis du synes det er irriterende, kan du se hvordan `display.launch.py` er skrevet og lage en lignende launch-fil selv.

URDF-filen skal ha en robot modell. Modellen skal bestå av 
- `world`: Den trenger ingen geometri. Dette er referanserammen til "robotcellen".
- `base_link`: Den trenger ingen geometri. Den er bunnen av modellen.
- `stator_link`: Det er en boks som er 102x102x102mm. Den skal være svart.
- `rotor_link`: Det er en sylinder som er 10mm høy og 50mm i diameter. Den skal være rød.
- `angle_link`: Det er en boks som er 25x2x2mm. Den skal være hvit.
Disse linkene skal kobles sammen via `joints`.
- `base_joint`: Fixed. Den kobler `world` og `base_link` og det er null transformasjon (`<origin xyz="0 0 0" rpy="0 0 0" >/`) mellom de.
- `stator_joint`: Fixed, og kobler `base_link` og `stator_link`. `base_link` skal være i bunnen av boksen, så leddet må koble `stator-link` til å være 102/2mm over `base_link` i z-retning.
- `angle`: Revolute, og kobler `stator_link` og `rotor_link`. `rotor_link` skal være på toppen av boksen, så leddet må koble `stator_link` til å være 102/2mm under `rotor_link` i z-retning.
- `indicator`: Fixed, og kobler `rotor_link` og `angle_link`. Du skal plassere `angle_link` slik at rektangelet stikker litt opp fra sylinderen, og at den går fra midten av sylinderen til ytterkanten.
Du kan se hvordan den skal se ut under
![bilde av modellen](https://github.com/adamleon/ais2105/blob/main/lab_assignments/lab_3/model.png?raw=true)
# Oppgave 2
## Del A
Du skal nå bruke URDF-filen fra oppgave 1 del B. Sørg for at den ligger i urdf-mappen i pakken din. Du skal endre launch-filen til å åpne URDF-filen, lese den via xacro og bruke innholdet til parameter til robot state publisher. Koden skal være noe lignende dette
```python
import os
import xacro

  # Use xacro to process the file
xacro_file = os.path.join(get_package_share_directory("joint_description"),"urdf","joint_model.urdf")
    robot_description_content = xacro.process_file(xacro_file).toxml()   
```

Du har også kanskje lagt merke til at hver gang du starter RViz at den resetter og at du må konfigurere den hver gang. Du skal lage en konfigurasjonsfil som lastes inn når RViz starter opp.

Først starter du opp RViz og gjør de konfigurasjonene du ønsker (RobotModel, osv.). Så går du på File og Save Config As..., og lagrer filen inne i en ny mappe som heter config, inni `joint_description`-mappen.

I launch-filen legger du til en parameter til RViz-noden.
```python
Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', [os.path.join(get_package_share_directory("joint_description"), 'config', '<navn på fil>.rviz')]]
)
```
Husk å endre `setup.py` eller `CMakeLists.txt` slik at du flytter `config` til `share`-mappen (på samme måte som du gjorde med `launch`-mappen for å kunne kjøre `ros2 launch joint_description ...`).

Tilslutt skal du starte en joint state publisher GUI node (executable `joint_state_publisher_gui`, package `joint_state_publisher`). Den tar robotens tilstand fra robot state publisher og kan styre leddene via en `/joint_states` topic. Kjører du `rqt_graph` kan du se hvordan joint state publisher og robot state publisher henger sammen.

## Del B
Det siste du skal gjøre er å bruke xacro til å forbedre URDF-filen. Dette gjør det mulig å lage variabler, funksjoner og gjøre utregninger i URDF-filen. Dette skal vi bruke til å parametrisere URDF-filen.

Gå gjennom tutorialen om xacro [her](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html). Bruk dette til å lag om dimensjonene i URDF-en (102mm, 50mm, osv) til `xacro: property`. For eksempel:
```xml
<xacro:property name="box_width" value="0.102" />
```
Du skal selvfølgelig også bruke disse variablene i resten av URDF-en istedenfor hardkodede tall.

En ting du kan legge merke til i mange [description pakker](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/rolling/urdf) er at man har to eller flere URDF-er, hvor en heter noe med makro. Hvorfor? Tenk deg at du skal lage en URDF for en robot-celle, med to UR5-roboter. Hvis du skulle gjort det med kun URDF, måtte du skrevet alle links og joints dobbelt opp med unike navn. Et eksempel er dette her:
```xml
<robot>
<link name="world"/> <!--Definere ʷΣ-->
<link name="ur1_base_link"> <!--Definere ᵘʳ¹Σ-->
	<geometry> ...
</link>
<link name="ur1_shoulder_link"> <!--Definere ᵘʳ¹ˢΣ-->
	<geometry> ...
</link>
... ovs for alle links til ur1
<joint name="ur1_base_world_joint" > <!--Definere transformasjonen mellom ʷΣ og ᵘʳ¹Σ-->
	<parent name="world" />
	<child name="ur1_base_link" />
	<origin xyz="1 0 0" />
</joint>
... osv for alle joints ur1

<link name="ur2_base_link"> <!--Definere ᵘʳ²Σ-->
	<geometry> ...
</link>
<link name="ur2_shoulder_link"> <!--Definere ᵘʳ²ˢΣ-->
	<geometry> ...
</link>
... ovs for alle links til ur2
<joint name="ur2_base_world_joint" > <!--Definere transformasjonen mellom ʷΣ og ᵘʳ²Σ-->
	<parent name="world" />
	<child name="ur2_base_link" />
	<origin xyz="-1 0 0" />
</joint>
... osv for alle joints ur2
</robot>
```
Lager man istedenfor en makro av hele roboten *og* den makroen tar inn en `prefix`-parameter, vil man kunne kopiere en robot så mange ganger man vil. Her er et eksempel:
```xml
<robot>
<link name="world"/> <!--Definere ʷΣ-->
<xacro:ur_robot prefix="ur1_" /> <!--Instansier en robot som heter ur1 -->
<xacro:ur_robot prefix="ur2_" /> <!-- Instansier en robot som heter ur2 -->
 
<joint name="ur1_world_joint" > <!--Definer transformasjonen mellom ʷΣ og ᵘʳ¹Σ -->
	<parent name="world" />
	<child name="ur1_base_link" />
	<origin xyz="1 0 0" />
</joint>
<joint name="ur2_world_joint" > <!--Definer transformasjonen mellom ʷΣ og ᵘʳ²Σ -->
	<parent name="world" />
	<child name="ur2_base_link" />
	<origin xyz="-1 0 0" />
</joint>

<!--Definer en makro som heter ur_robot med en parameter som heter prefix. prefix legges til alle links og joints for å få unike navn -->
<xacro:macro name="ur_robot" params="prefix">
	<link name="${prefix}base_link">
		<geometry> ...
	</link>
	<link name="${prefix}shoulder_link">
		<geometry> ...
	</link>
	... ovs for alle links til en UR5
	<joint name="${prefix}base_shoulder_joint" >
		<parent name="${prefix}base_link" />
		<child name="${prefix}shoulder_link" />
		<origin xyz="${shoulder_length} 0 0" />
	</joint>
	... osv for alle joints til en UR5
</xacro:macro>
</robot>
```
Som du ser så brukes makro til å beskrive hvordan roboten ser ut. Det som vanligvis gjøres er at du har en separat makro-URDF-fil som beskriver roboten (som link, joint, osv.) og dette skrives som en makro. Du ser eksempel på det [her](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/rolling/urdf/ur_macro.xacro). Man kan da kalle på makro-URDF-en i fra en "default"-URDF-fil som vist [her](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/rolling/urdf/ur.urdf.xacro).

Ok. Nok snakk. Du skal nå flytte alt av links, joints, osv. (bortsett fra world linken og base_joint jointen) å lage en makro av det i en separat fil du kaller `joint_model.macro.urdf`. Makroen skal hete `joint_model` og ha en `params="prefix"`. Bruk deretter `${prefix}` foran alle navnene på links og joints.

Tilbake i `joint_model.urdf` kaller du `<xacro:include filename="$(find joint_description)/urdf/joint_model.macro.urdf"/>` for å laste inn makro-filen, og kaller `<xacro:joint_model prefix="" />` lengre ned i koden. Se over at `world` linken og `base_link` (som nå er definert inni makroen) har null transformasjon (`xyz="0 0 0" rpy="0 0 0"`).

## Del A og B
Dere kan nå leke dere med GUI og se at modellen i RViz flytter seg når du endrer på vinklene i GUI.

Hvis dere kjører 
```
colcon build --symlink-install
```
så vil dere lage en symbolsk lenke mellom URDF-ene og de installerte URDF-ene (de som blir kopiert over til `share`-mappen). Dette gjør at alle endringer du gjør i URDF blir automatisk oppdatert uten å måtte bygge hver gang.

I URDF-en (altså ikke makro-URDF-en) kan dere leke med å flytte modellen rundt ved å endre transformasjonen mellom world og base_link. Dere kan også endre prefix og lage flere modeller (og definere joints for de separat). Dere vil se at hvis dere legger til flere modeller vil også GUI-en få flere ledd dere kan styre. 
