# Firmware ESP32 – Capteurs de cuves (TF-Luna)

Ce dépôt contient le firmware des capteurs de cuves basés sur ESP32 + TF-Luna pour le Château Lamothe Despujols.

- Lecture de distance TF-Luna (UART2, GPIO16/17)
- Calcul du volume / % de remplissage des cuves
- Envoi périodique des mesures en JSON HTTPS vers le serveur Infomaniak  
  (`https://prod.lamothe-despujols.com/cuves/api_cuve.php`)
- Récupération de la configuration cuve depuis  
  `https://prod.lamothe-despujols.com/cuves/get_config.php`
- Configuration Wi-Fi via WiFiManager (AP `Cuve_Config_AP` si besoin)
- Mise à jour OTA centralisée via `ota_check.php` (`firmware.bin` hébergé sur le serveur)

Le dashboard web associé se trouve dans ce dépôt séparé :  
?? https://github.com/ChtLam33/DashboardWeb
