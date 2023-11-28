# tfg-overtaking

### Repositorio

```python
https://github.com/llanahp/tfg-overtaking
```

### Clonar

```python
git clone https://github.com/llanahp/tfg-overtaking overtaking
cd overtaking/src
```

### Ejecutar solo linea

- Ejecutar  la misma acción siempre
    
    ```bash
    python3 test.py --env=overtakingLine --eval=True
    ```
    
- Ejecutar con un entrenamiento
    
    Se debe cambiar la fecha por la del entrenamiento previo
    
    ```bash
    python3 test.py --env=overtakingLine --scenario='2023-11-12 17:46:16.055945'
    ```
    
- Entrenar
    
    ```bash
    python3 train.py --env=overtakingLine 
    ```
    
- Entrenar partiendo de un entrenamiento previo
    
    Se debe cambiar la fecha por la del entrenamiento previo
    
    ```bash
    python3 train.py --env=overtakingLine --pretrained='2023-11-15 10:07:31.764785'
    ```
    

---

### Ejecutar circulo completo

- Ejecutar  la misma acción siempre
    
    ```bash
    python3 test.py  --eval=True
    ```
    
- Ejecutar con un entrenamiento
    
    Se debe cambiar la fecha por la del entrenamiento previo
    
    ```bash
    python3 test.py --scenario='2023-11-12 17:46:16.055945'
    ```
    
- Entrenar
    
    ```bash
    python3 train.py 
    ```
    
- Entrenar partiendo de un entrenamiento previo
    
    Se debe cambiar la fecha por la del entrenamiento previo
    
    ```bash
    python3 train.py  --pretrained='2023-11-15 10:07:31.764785'
    ```
    

---

### Listar entrenamientos

```bash
ls -lta Training/Models/
```
