set "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
set "HOST_ADDR=1.2.3.4"

REM Using participant index
set "CYCLONEDDS_URI=<CycloneDDS><Domain id='any'><General><NetworkInterfaceAddress>${HOST_ADDR}</NetworkInterfaceAddress><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>0</ParticipantIndex><Peers><Peer address='${HOST_ADDR}:7412'/></Peers></Discovery><Tracing><Verbosity>config</Verbosity><Out>stderr</Out></Tracing></Domain></CycloneDDS>"