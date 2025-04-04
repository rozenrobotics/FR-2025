# Репозиторий с решением командной задачи профиля "Летающая робототехника" за 2024-2025гг.


## Установка и сборка

> Предполагается, что образ симулятора уже установлен на вашей машине вместе с необходимыми зависимостями (catkin_make, rosdep и т.п).
> Если это не так, то воспользуйтесь [этим туториалом](https://clover.coex.tech/ru/simulation_native.html) для первичной установки.
>
> Также вы можете использовать [докер-контейнер](), в котором уже установлен симулятор с зависимостями, а также собран и подготовлен проект из этого репозитория. Команды для запуска смотрите в разделе ["Запуск docker-контейнера"](#docker-container). 

#### Шаг 1 (Создание рабочего окружения)
Если у вас уже есть рабочее окружение `~/catkin_ws`, то можете пропустить этот шаг и переходить к следующему.
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

#### Шаг 2 (Клонирование репозитория, инициализация подмодулей)
```bash
cd ~/catkin_ws/src
git clone --recurse-submodules https://gitlab.com/Artemy2807/fr-team-task-rozen
```

#### Шаг 3 (Установка зависимостей)
```bash
sudo apt-get update && sudo apt-get install -y xmlstarlet
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
/usr/bin/python3 -m pip install -r src/fr-team-task-rozen/offboard/app/requirements.txt
```

#### Шаг 4 (Сборка проекта, настройка образа симулятора)
```bash
cd ~/catkin_ws
catkin_make
./src/fr-team-task-rozen/setup.bash
source ~/.bashrc
```


## Описание программных модулей

### Пакет для генерации мира - `world_generator`
Пакет генерирует сцену со случайно расположенными dronepoint`ами в количестве 5 штук. Цвет каждого dronepoint'а выбирается случайно.

#### Пример команды для генерации случайного мира
```bash
rosrun world_generator generate.py
```

#### Пример вывода при успешном выполнении скрипта:
```
Generated dronepoints:
	model=dronepoint_yellow; x=2.21; y=2.44
	model=dronepoint_yellow; x=5.56; y=1.54
	model=dronepoint_green; x=6.49; y=4.78
	model=dronepoint_yellow; x=6.5; y=7.79
	model=dronepoint_yellow; x=1.06; y=6.29
Creating file:
	/root/catkin_ws/src/clover/clover_simulation/resources/worlds/clover_aruco.world
Clover world created successfully
```

> !ВАЖНО! После запуска скрипта, мир `clover_simulation/launch/clover_aruco.world` будет перезаписан. При следующем запуске симулятора, сгенерированный мир будет подгружен автоматически.


### Пакет автономного выполнения миссии и мониторинга - `offboard`
Данный пакет содержит скрипт `mission.py`, которые отвечает за автономное выполнение миссии. Запускать данный скрипт в ручную не требуется. Это можно сделать через веб-интерфейс мониторинга.

#### Запуск веб-интерфейса мониторинга
> !Перед запуском веб-интерфейса необходимо запустить сцену в симуляторе:
> ```bash
> roslaunch clover_simulation simulator.launch
> ```

```bash
cd ~/catkin_ws/src/fr-team-task-rozen/offboard/app
flask run
```

После запуска вы сможете открыть интерфейс в браузере по адресу `http://127.0.0.1:5000`.

#### Скриншот интерфейса мониторинга
![Web](resources/web.jpg?raw=true)


## Запуск docker-контейнера<a name="docker-container"></a>

```bash
docker login registry.gitlab.com

docker run -it --rm -e DISPLAY=unix$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/xenial:/home/xenial --env="DISPLAY" --network="host"  --device=/dev/dri:/dev/dri registry.gitlab.com/artemy2807/fr-team-task-rozen
```


## Демонстрация выполнения командной задачи
[![Rutube](resources/preview.jpg?raw=true)](https://rutube.ru/video/private/a11023437ef5d8bbfa5a9cda7da932e9/?p=rw6NwMqsMYkWeiY2iBEfqA)


## Источники кода
- Веб портирован с https://github.com/Tennessium/HUEX
- Динамическая генерация SVG изображений Aruco-меток - https://chev.me/arucogen/

## Контакты
- Одышев Артемий - [Telegram](https://t.me/art_2807)
