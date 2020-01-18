docker build -f Dockerfile_trl -t rlframe_trl:latest .

docker run --rm -it rlframe_trl:latest /bin/bash -c "pushd /root/playground/TerrainRLSim; git pull origin master; python3 simAdapter/terrainRLSimTestNoRender.py"
