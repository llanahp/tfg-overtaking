# tfg-overtaking

### Clonar

```python
git clone https://github.com/llanahp/tfg-overtaking overtaking
cd overtaking/src
git checkout sumo
```

### Ejecutar adelantamiento en linea
```
python3 test.py --env=Highway --model=TRPO --compPretrained='Highway_TRPO_2023-12-28 10:45:29.554931'
python3 test.py --env=HighwaySac --model=SAC --compPretrained='Highway_SAC_2023-12-25 22:41:01.794337'

python3 test.py --env=overtakingOneLinePPO --model=PPO --compPretrained="overtakingOneLine_PPO_2023-12-28 14:29:24.831595"
python3 test.py --env=overtakingOneLineSac --model=SAC --compPretrained="overtakingOneLine_SAC_2023-12-27 23:34:06.892096"
python3 test.py --env=overtakingOneLine --model=TRPO --compPretrained="overtakingOneLine_TRPO_2023-12-28 12:01:34.821308"
```
---

### entrenar
    python3 train.py --env=overtakingOneLine --model=TRPO
---

### Listar entrenamientos

```bash
ls -lta Training/Models/
```
---


https://opalescent-berry-180.notion.site/TFG-a28e3eb61a74451b8a41e2f7a5a4a2a0?pvs=4
