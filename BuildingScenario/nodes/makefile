all: app

app:
	@export SUMO_HOME=/home/sumo-0.25.0/sumo/
	@netconvert -n cenario.nod.xml -e cenario.edg.xml -o cenario.net.xml
	@duarouter -n cenario.net.xml -f flow.xml -o cenario.rou.xml
	@sumo -c cenario.sumo.cfg --fcd-output cenarioTrace.xml
	@/usr/local/src/sumo-0.25.0/tools/traceExporter.py --fcd-input cenarioTrace.xml --ns2mobility-output ns2mobility.tcl

