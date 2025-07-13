# Instructions de compilation pour le composant BNO055

## Méthode 1 : Compilation directe avec ESPHome

### 1. Préparer votre configuration
```bash
# Créer un nouveau projet ESPHome
esphome init bno055_project
cd bno055_project
```

### 2. Copier les fichiers
```bash
# Copier vos fichiers dans le projet
cp ../bno055.h components/
cp ../bno055.cpp components/
cp ../manifest.yaml .
```

### 3. Créer la configuration YAML
```yaml
# configuration.yaml
esphome:
  name: bno055_test
  friendly_name: BNO055 Test

esp32:
  board: esp32dev
  framework:
    version: latest

# Inclure le composant externe
external_components:
  - source: ./
    components: [ bno055 ]

i2c:
  sda: 21
  scl: 22
  frequency: 400kHz

sensor:
  - platform: bno055
    i2c_id: i2c_bus
    address: 0x28
    update_interval: 50ms
    
    compass:
      name: "Compass Heading"
      unit_of_measurement: "°"
      accuracy_decimals: 1
```

### 4. Compiler
```bash
esphome compile configuration.yaml
```

## Méthode 2 : Compilation avec ESPHome Dashboard

### 1. Ouvrir ESPHome Dashboard
```bash
esphome dashboard
```

### 2. Créer un nouveau projet
- Cliquer sur "+" pour créer un nouveau projet
- Nommer le projet "bno055_test"

### 3. Ajouter le composant externe
Dans la configuration YAML, ajouter :
```yaml
external_components:
  - source: ./
    components: [ bno055 ]
```

### 4. Ajouter la configuration complète
Copier le contenu de `bno055.yaml` dans l'éditeur

### 5. Compiler
- Cliquer sur "Compile" dans l'interface web

## Méthode 3 : Compilation locale avec PlatformIO

### 1. Structure du projet
```
bno055_project/
├── components/
│   ├── bno055/
│   │   ├── bno055.h
│   │   ├── bno055.cpp
│   │   └── manifest.yaml
├── configuration.yaml
└── platformio.ini
```

### 2. Créer platformio.ini
```ini
[platformio]
src_dir = .

[env:esp32dev]
platform = espressif32
board = esp32dev
framework = esphome
monitor_speed = 115200
```

### 3. Compiler
```bash
pio run
```

## Dépannage

### Erreurs courantes :
1. **Composant non trouvé** : Vérifier que `manifest.yaml` est dans le bon dossier
2. **Erreur de compilation** : Vérifier les dépendances ESPHome
3. **Erreur I2C** : Vérifier les pins SDA/SCL

### Vérifications :
```bash
# Vérifier la version ESPHome
esphome version

# Vérifier la configuration
esphome validate configuration.yaml
```

## Upload et test

### 1. Uploader le firmware
```bash
esphome upload configuration.yaml
```

### 2. Monitorer les logs
```bash
esphome logs configuration.yaml
```

### 3. Vérifier les données
- Ouvrir ESPHome Dashboard
- Vérifier que les capteurs apparaissent
- Tester les lectures du compas 