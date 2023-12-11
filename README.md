# tfg-overtaking

### Clonar

```python
git clone https://github.com/llanahp/tfg-overtaking overtaking
cd overtaking/src
```

### Ejecutar adelantamiento en linea
    python3 test.py --env=overtakingOneLine --scenario='2023-12-06 10:47:11.692484'
---

### entrenar
    python3 train.py --env=overtakingOneLine --model=TRPO
---

### Listar entrenamientos

```bash
ls -lta Training/Models/
```
---

### Mejores entrenamientos
- PPO
    
    ```bash
    python3 test.py --env=overtakingLine --model=PPO --scenario='2023-12-01 09:47:57.490670'
    ```
    
- A2C
    
    ```bash
    python3 test.py --env=overtakingLine --model=A2C --scenario='2023-11-30 20:31:15.657624'
    ```
    
- SAC
    
    ```bash
    #cambiar _actions y __init__
    python3 test.py --env=overtakingLine --model=SAC --scenario='2023-11-30 17:13:29.847763'
    ```
    
- DQN 
    
    ```bash
    python3 test.py --env=overtakingLine --model=DQN --scenario='2023-11-30 18:34:55.556011'
    ```
