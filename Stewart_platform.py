import json
import time
from os import environ

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import serial
import serial.tools.list_ports
import simple_pid.PID as PID
from matplotlib.widgets import Button, Slider
from mpl_toolkits.mplot3d import Axes3D
from numpy import linalg as LA
from scipy import signal
from scipy.spatial.transform import Rotation as R



# !!! Oznaczenia serwomechanizmów stosowane  w oprogramowaniu różni się od oznaczeń stosowanych na schematach elektrycznych zawartych w pracy inżynierskiej !!!
# !!! Oznaczenia w oprogramowaniu: S1, S2, S3, S4, S5, S6 !!!
#         Binek
#        ________
#       /        \
#      /S2      S3\
#     /            \    Ochowiak
#    /S1          S4\
#   /                \
#   \                /  Mojs
#    \___S6____S5___/
#     |230V| |D-Sub|

# !!! Oznaczenia na schematach elektrycznych w pracy inżynierskiej: S6, S5, S4, S3, S2, S1 !!!
#         Binek
#        ________
#       /        \
#      /S5      S4\
#     /            \    Ochowiak
#    /S6          S3\
#   /                \
#   \                /  Mojs
#    \___S1____S2___/
#     |230V| |D-Sub|




# Parametry mechaniczne konstrukcji
a = 29      # Długość orczyków
s = 124     # Długość popychaczy
betai = [   
            0.0, 
            180 + 0.0, 
            -119.95360816780149, 
            180 - 119.95360816780149, 
            119.95360816780149, 
            180 + 119.95360816780149
        ]   # Kąt określający rozmieszczenie serwomechanizmów 

# Położenie kątowe w serwomechanizmach, obliczone z zadania odwrotnego kinematyki
alfai = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Położenie kątowe w serwomechanizmach, w poziomej pozycji stołu - Wartość musi być kalibrowano po przestawieniu platrformy w inne miejsce
# Serwomechanizmy (oznaczenie stosowane w oprogramowaniu): S1, S2, S3, S4, S5, S6
# Orczyki serwomechanizmów (S1,S2), (S3,S4), (S5,S6) leżą w tej samej płaszczyźnie, jednak mają przeciwny zwrot.
# Zwiększanie wartości kąta alfai_calibration powoduje podniesienie orczyka w S2, S4, S6
# Zmniejszanie wartości kąta alfai_calibration powoduje podniesienie orczyka w S1, S3, S5
alfai_calibration  = [90, 90, 96, 89, 87, 87]

# Wartość położenia kątowego przekazana do Arduino
alfa1 = alfai_calibration[0]
alfa2 = alfai_calibration[1]
alfa3 = alfai_calibration[2]
alfa4 = alfai_calibration[3]
alfa5 = alfai_calibration[4]
alfa6 = alfai_calibration[5]

# Rozmiar tabeli historii sygnałów regulatorów wyświetlanych na wykresach przebiegu sygnałów sterujących
pid_signal_array_size = 1000
# Indeks tabeli historii sygnałów regulatorów
pid_signal_array_idx = 0

# Liczba punktów składających się na trajektorię 
trajectory_array_size = 32
# Indeks punktu leżącego na trajektorii kulki (punkt, do którego zmierza kulka)
trajectory_array_idx = 0
# odległość w mm, na którą musi zbliżyć się kulka, aby zmienić punkt docelowy na następny z tabeli punktów składających się na trajektorię
margin = 20

# Rozmiar tabeli historii odczytanych pozycji kulki
max_buffor_pointer = 25

# Położenie wierzchołków panelu dotykowego wyrażone w lokalnym układzie współrzędnych
# Lokalny układ współrzędnych - układ zaczepiony do panelu, układ ruchomy
# Globalny układ współrzędnych - układ związany z nieruchomą podstawą, układ nieruchomy
Panel_init_not_rotated = np.array([
  [-115, -150, 0], 
  [-115,  150, 0],
  [ 115,  150, 0], 
  [ 115, -150, 0],
  [-115, -150, 0]]
)

r_rotate_panel = R.from_euler('zyx', [ -30, 0, 0 ], degrees=True)
r_int_rotate_panel = R.from_euler('zyx', [ 30, 0, 0 ], degrees=True)

# Położenie wierzchołków panelu dotykowego wyrażone w globalnym układzie współrzędnych
Panel_init = r_rotate_panel.apply(Panel_init_not_rotated)

# Położenie wierzchołków panelu dotykowego z uwzględnieniem dodatkowego przesunięcia w kierunku osi globalnego układu współrzędnych
# Dodatkowe przesunięcie  w kierunku  osi globalnego układu współrzędnych, wprowadzane jest tylko w trybie manualnym
Panel = np.zeros((5,3))

# Początkowe położenie wierzchołków pomarańczowego kwadratu symbolizującego kulkę (widoczny na wykresie po prawej stronie interfejsu)
Ball_box_init = np.array([
    [-5, -5, 0],
    [-5,  5, 0],
    [ 5,  5, 0],
    [ 5, -5, 0],
    [-5, -5, 0]
])

# Położenie wierzchołków pomarańczowego kwadratu symbolizującego kulkę
Ball_box = np.zeros((5,3))

# Początkowe położenie punktów umieszczenia serwomechanizmu
Bi_init = np.array(
    [
        [ -34,  95, 0],
        [  34,  95, 0],  
        [ 100, -16, 0],
        [  66, -75, 0], 
        [ -66, -75, 0],
        [-100, -16, 0],
        [ -34,  95, 0]
    ]
)

# Położenie punktów umieszczenia serwomechanizmu
Bi = np.zeros((7,3))

# Początkowe położenie punktów połączenia popychaczy i panelu dotykowego
Pi_init = np.array(
    [
        [-65,  67, 0],
        [ 65,  67, 0],
        [ 93,  25, 0],
        [ 25, -90, 0],
        [-25, -90, 0],
        [-93,  25, 0],
        [-65,  67, 0]
    ]
)

# Położenie punktów połączenia popychaczy i panelu dotykowego
Pi = np.zeros((7,3))

# Wektory położenia, prędkości i przyspieszenia kulki
ball_pos = np.array([0.0, 0.0, 0.0])
ball_vel = np.array([0.0, 0.0, 0.0])
ball_acc = np.array([0.0, 0.0, 0.0])
ball_pos_rot = np.array([0.0, 0.0, 0.0])

Orczyk = np.zeros((6,2,3))

# Generowanie punktów składowych trajektorii
x = np.linspace(0, 2 * np.pi, trajectory_array_size)
trajektoria = np.zeros((trajectory_array_size, 3))
trajektoria[:,0] = 80*np.sin(x)
trajektoria[:,1] = 80*np.cos(x)

# Wyrażenie trajektorii w globalnym układzie współrzędnych
# Przyjmujemy, że trajektoria się nie obraca wraz z panelem
trajektoria_g = r_rotate_panel.apply(trajektoria)
ball_target = trajektoria_g[0,:]

# Punkty symbolizujące granicę panelu, rysowane na wykresie po prawej stronie
points = np.zeros((42+trajectory_array_size, 2))
points[2:12,0] = np.linspace(Panel_init_not_rotated[0,0],Panel_init_not_rotated[1,0], 10)
points[2:12,1] = np.linspace(Panel_init_not_rotated[0,1],Panel_init_not_rotated[1,1], 10)
points[12:22,0] = np.linspace(Panel_init_not_rotated[1,0],Panel_init_not_rotated[2,0], 10)
points[12:22,1] = np.linspace(Panel_init_not_rotated[1,1],Panel_init_not_rotated[2,1], 10)
points[22:32,0] = np.linspace(Panel_init_not_rotated[2,0],Panel_init_not_rotated[3,0], 10)
points[22:32,1] = np.linspace(Panel_init_not_rotated[2,1],Panel_init_not_rotated[3,1], 10)
points[32:42,0] = np.linspace(Panel_init_not_rotated[3,0],Panel_init_not_rotated[4,0], 10)
points[32:42,1] = np.linspace(Panel_init_not_rotated[3,1],Panel_init_not_rotated[4,1], 10)

# Punkty symbolizujące przebieg trajektorii, rysowane na wykresie po prawej stronie
points[42:42+trajectory_array_size,0] = trajektoria[:,0]
points[42:42+trajectory_array_size,1] = trajektoria[:,1]

# Oś pozioma wykresu historii sygnałów regulatorów
t = np.linspace(0, pid_signal_array_size-1, pid_signal_array_size)

# Tabele historii sygnałów regulatorów
p_pid_component_array = np.zeros(pid_signal_array_size)  # Część proporcjonalna
d_pid_component_array = np.zeros(pid_signal_array_size)  # Część różniczkująca
error_pid_array = np.zeros(pid_signal_array_size)        # Sygnał błędu

# Dodawanie trzech głównych wykresów
fig = plt.figure(figsize=(18, 8))
ax1 = fig.add_subplot(131, projection='3d')
ax2 = fig.add_subplot(132)
ax3 = fig.add_subplot(133)

# Przekazanie danych do wykresów
data = [Panel_init, Ball_box_init, Bi_init, Pi_init, Orczyk[0,:,:], Orczyk[1,:,:], Orczyk[2,:,:], Orczyk[3,:,:], Orczyk[4,:,:], Orczyk[5,:,:]]
lines1 = [ax1.plot(dat[0:1, 0], dat[0:1, 1], dat[0:1, 2])[0] for dat in data]
lines2 = [ax2.plot(p_pid_component_array)[0], ax2.plot(d_pid_component_array)[0], ax2.plot(error_pid_array)[0]]
lines3 = [ax3.scatter(points[:,0], points[:,1])]
line = [lines1, lines2, lines3]

# Ustawienie opisów osi
ax1.set_xlim3d([-150.0, 150.0])
ax1.set_xlabel('X')
ax1.set_ylim3d([-150.0, 150.0])
ax1.set_ylabel('Y')
ax1.set_zlim3d([0.0, 225.0])
ax1.set_zlabel('Z')
ax2.set_ylim([-3, 3])
ax2.legend(('p','d', 'error'), loc="upper right")
ax3.set_xlim([-120, 120])
ax3.set_ylim([-160, 160])

axcolor = 'lightgoldenrodyellow'

# Dodanie sliderów
plt.subplots_adjust(bottom=0.25)
# Umiejscowienie sliderów na ekranie
axX = plt.axes([0.325, 0.18, 0.15, 0.03], facecolor=axcolor)
axY = plt.axes([0.325, 0.13, 0.15, 0.03], facecolor=axcolor)
axZ = plt.axes([0.325, 0.08, 0.15, 0.03], facecolor=axcolor)
axEulerR = plt.axes([0.55, 0.18, 0.15, 0.03], facecolor=axcolor)
axEulerP = plt.axes([0.55, 0.13, 0.15, 0.03], facecolor=axcolor)
axEulerY = plt.axes([0.55, 0.08, 0.15, 0.03], facecolor=axcolor)

# Przesunięcie w kierunku osie globalnego układu współrzędnych
X = 0.0
Y = 0.0
Z = 134.25

# Kąty obroty wokół osi globalnego układu współrzędnych
EulerR = 0.0
EulerP = 0.0
EulerY = 0.0

sX = Slider(axX, 'X', -50.0, 50.0, valinit=X)
sY = Slider(axY, 'Y', -50.0, 50.0, valinit=Y)
sZ = Slider(axZ, 'Z', 100.0, 150.0, valinit=Z)
sEulerR = Slider(axEulerR, 'Kąt R', -18.0, 18.0, valinit=EulerR)
sEulerP = Slider(axEulerP, 'Kąt P', -18.0, 18.0, valinit=EulerP)
sEulerY = Slider(axEulerY, 'Kąt Y', -18.0, 18.0, valinit=EulerY)

def reset(event):
    global ball_pos, ball_vel, ball_acc
    sX.reset()
    sY.reset()
    sZ.reset()
    sEulerR.reset()
    sEulerP.reset()
    sEulerY.reset()
    ball_pos = np.array([0.0, 0.0, 0.0])
    ball_vel = np.array([0.0, 0.0, 0.0])
    ball_acc = np.array([0.0, 0.0, 0.0])

resetax = plt.axes([0.8, 0.01, 0.1, 0.04])
button = Button(resetax, 'Reset', color=axcolor, hovercolor='0.975')
button.on_clicked(reset)

# Macierz rotacji RPY
r_init = R.from_euler('zyx', [ sEulerY.val, sEulerP.val, sEulerR.val ], degrees=True)

# Filtr butterwortha stosowany do wygładzania sygnału pozycji kulki
butter_filter = signal.butter(2, 2.5, 'low', fs=25, output='sos')
# Bufor filtrowanego sygnału położenia kulki
ball_pos_buffor = np.zeros((max_buffor_pointer, 2))
# Odfiltrowany sygnał położenia kulki
filtered = np.zeros((max_buffor_pointer, 2))
# Mediana trzech ostatnich odczytów położenia kulki
pos_mediana = np.zeros((1, 2))
# Mediana 4, 5 i 6 odczytu położenia kulki
# Róznica median traktowana jest do obliczenia prędkości kulki
prev_pos_mediana = np.zeros((1, 2))

# Kwadrat maksymalnego przemieszczenia piłki
# Pomiary nie przekraczające tą wartość zostają odrzucone
max_step = 2000

# Parametry regulatora PID
pid_X = PID(0.035, 0.0, 0.025, Td=0.2, max_derivative_change=0.2, setpoint=0.0)
pid_Y = PID(0.035, 0.0, 0.025, Td=0.2, max_derivative_change=0.2, setpoint=0.0)
pid_X.proportional_output_limits = (-0.75, 0.75)
pid_Y.proportional_output_limits = (-0.75, 0.75)
pid_X.derivative_output_limits = (-1, 1)
pid_Y.derivative_output_limits = (-1, 1)
pid_X.output_limits = (-0.5, 0.5)
pid_Y.output_limits = (-0.5, 0.5)
pid_X.sample_time = 0.035
pid_Y.sample_time = 0.035



def suppress_qt_warnings():
    environ["QT_DEVICE_PIXEL_RATIO"] = "0"
    environ["QT_AUTO_SCREEN_SCALE_FACTOR"] = "1"
    environ["QT_SCREEN_SCALE_FACTORS"] = "1"
    environ["QT_SCALE_FACTOR"] = "1"


# Funkcja obliczająca kinematykę odwrotną
def IK(B, P, beta, a=29.0, s=150.0):
  l = LA.norm(P-B)
  L = l**2-(s**2-a**2)
  M = 2.0*a*(P[2]-B[2])
  N = 2.0*a*((P[0]-B[0])*np.cos(np.pi*beta/180.0)+(P[1]-B[1])*np.sin(np.pi*beta/180.0))
  alfa = 180.0*(np.arcsin(L/np.sqrt(M**2 + N**2)) - np.arctan(N/M))/np.pi
  return -alfa


def URA_XY():
    global ball_acc, ball_vel, ball_pos, pid_signal_array_idx, x, t, Bi, start, ball_pos_rot, ball_target, trajectory_array_idx, trajektoria_g, r_init
        
    error = ball_target - ball_pos
    error_mod = np.sqrt(np.dot(error,error))
    if error_mod < margin:
        ball_target = trajektoria_g[trajectory_array_idx,:]
        error = ball_target - ball_pos
        error_mod = np.sqrt(np.dot(error, error))
        trajectory_array_idx += 1
        trajectory_array_idx %= trajectory_array_size

    sEulerR.val = pid_X(error[1])
    sEulerP.val = pid_Y(-error[0])
    
    r_init = R.from_euler('zyx', [ sEulerY.val, sEulerP.val, sEulerR.val ], degrees=True)
    p_pid_component_array[pid_signal_array_idx], id, d_pid_component_array[pid_signal_array_idx] = pid_X.components
    error_pid_array[pid_signal_array_idx] = np.power(10.0, -2)*np.sqrt(np.dot(error,error))


def Manual():
    global ball_acc, ball_vel, ball_pos, pid_signal_array_idx, x, t, Bi, start, ball_pos_rot, ball_target, trajectory_array_idx, trajektoria, r_init
    r_init = R.from_euler('zyx', [ sEulerY.val, sEulerP.val, sEulerR.val ], degrees=True)


def Physics_simulation():
    global ball_acc, ball_vel, ball_pos, pid_signal_array_idx, x, t, Bi, start, ball_pos_rot, ball_target, trajectory_array_idx, trajektoria, r_init, r_int_rotate_panel
    end = time.time()
    delta = end-start
    ball_acc = -981*r_init.as_dcm()[2,:]
    ball_acc[2] = 0.0
    ball_vel += delta*ball_acc
    ball_vel = np.power(0.5, delta)*ball_vel
    ball_pos += delta*ball_vel + 0.7*delta*delta*ball_acc
    ball_pos_rot = r_init.apply( ball_pos ) +  [sX.val, sY.val, sZ.val]
    start = time.time()
    points[0] = r_int_rotate_panel.apply(ball_pos)[0:2]
    pid_signal_array_idx += 1
    pid_signal_array_idx %= pid_signal_array_size


def Arduino_connection():
    global ball_acc, ball_vel, ball_pos, pid_signal_array_idx, x, t, Bi, start, ball_pos_rot, ball_target, trajectory_array_idx, trajektoria, r_init, r_int_rotate_panel,r_rotate_panel, ball_pos_buffor, filtered, max_step, pos_mediana, prev_pos_mediana
    end = time.time()
    delta = end-start
    start = time.time()
    json_sting = ""
    try:
        json_sting = str(ser.readline().decode("utf-8"))
        json_dic = json.loads(json_sting)
        ball_pos[0] = 115 - 230*(json_dic["y"]-50)/790
        ball_pos[1] = -150 + 300*(json_dic["x"]-35)/825
    except Exception as e:
        print("Error during loading JSON")
        print(json_sting)
        print(e)
    ser.flushInput()

    ball_pos_buffor = np.roll(ball_pos_buffor, -1, axis=0)
    ball_pos_buffor[max_buffor_pointer-1,:] = ball_pos[0:2]
    step = np.dot((ball_pos_buffor[max_buffor_pointer-1]-ball_pos_buffor[max_buffor_pointer-2]), (ball_pos_buffor[max_buffor_pointer-1]-ball_pos_buffor[max_buffor_pointer-2]))
    if step > 2000:
        ball_pos_buffor[max_buffor_pointer-1,:] = ball_pos_buffor[max_buffor_pointer-2,:]
        ball_pos[0] =  ball_pos_buffor[max_buffor_pointer-2,0]
        ball_pos[1] =  ball_pos_buffor[max_buffor_pointer-2,1]
        step = (ball_pos_buffor[max_buffor_pointer-2,0]-points[0][0])*(ball_pos_buffor[max_buffor_pointer-2,0]-points[0][0]) + (ball_pos_buffor[max_buffor_pointer-2,1]-points[0][1])*(ball_pos_buffor[max_buffor_pointer-2,1]-points[0][1])
    filtered[:,0] = signal.sosfilt(butter_filter, ball_pos_buffor[:,0])
    filtered[:,1] = signal.sosfilt(butter_filter, ball_pos_buffor[:,1])
    points[0][0] = filtered[max_buffor_pointer-1,0]
    points[0][1] = filtered[max_buffor_pointer-1,1]

    pos_mediana = np.median(filtered[22:25, :], axis=0)
    prev_pos_mediana = np.median(filtered[20:22, :], axis=0)
    ball_vel = pos_mediana - prev_pos_mediana
    ball_vel /= 0.04
    ball_pos[0:2] = points[0]
    ball_pos = r_rotate_panel.apply(ball_pos)
    ball_pos_rot = r_init.apply(ball_pos) +  [sX.val, sY.val, sZ.val]

    pid_signal_array_idx += 1
    pid_signal_array_idx %= pid_signal_array_size

    alfa1 = alfai_calibration[0] + int(alfai[0])
    alfa2 = alfai_calibration[1] - int(alfai[1]) 
    alfa3 = alfai_calibration[2] + int(alfai[2])
    alfa4 = alfai_calibration[3] - int(alfai[3])
    alfa5 = alfai_calibration[4] + int(alfai[4])
    alfa6 = alfai_calibration[5] - int(alfai[5])

    json_sting = "{{\"a1\":{0},\"a2\":{1},\"a3\":{2},\"a4\":{3},\"a5\":{4},\"a6\":{5}}}".format(
        alfa1, alfa2, alfa3, alfa4, alfa5, alfa6
    )
    print(json_sting)

    ser.write(bytes(json_sting, 'utf-8'))


def animate(j):
    global lines1, lines2, line, r_init, Panel, Ball_box, alfai
    #URA_XY()
    Physics_simulation()
    Manual()
    #Arduino_connection()    
    
    Pi = r_init.apply( Pi_init )
    Pi[:,0] = Pi[:,0] + sX.val
    Pi[:,1] = Pi[:,1] + sY.val
    Pi[:,2] = Pi[:,2] + sZ.val

    for j in range(6):
        alfai[j] = IK(Bi_init[j], Pi[j], betai[j])

    if pid_signal_array_idx%3==0:
        Panel = r_init.apply( Panel_init )
        Panel[:,0] = Panel[:,0] + sX.val
        Panel[:,1] = Panel[:,1] + sY.val
        Panel[:,2] = Panel[:,2] + sZ.val
        Panel = Panel.transpose()
        line[0][0].set_data(Panel[0:2,:])
        line[0][0].set_3d_properties(Panel[2,:])

        Ball_box = r_init.apply(Ball_box_init) + ball_pos_rot
        Ball_box = Ball_box.transpose()
        line[0][1].set_data(Ball_box[0:2,:])
        line[0][1].set_3d_properties(Ball_box[2,:])

        Bi = Bi_init.transpose()
        line[0][2].set_data(Bi[0:2,:])
        line[0][2].set_3d_properties(Bi[2,:])

        Pi = Pi.transpose()
        line[0][3].set_data(Pi[0:2,:])
        line[0][3].set_3d_properties(Pi[2,:])

        for j in range(6):
            r = R.from_euler('yz', [ alfai[j], betai[j]], degrees=True)
            v = [a, 0, 0]
            temp = r.apply(v) + Bi_init[j]
            temp = temp.transpose()
            Orczyk = np.stack((Bi[:,j], temp), axis=-1)
            temp  = np.array([Pi[:,j]]).transpose()
            Orczyk = np.concatenate((Orczyk, temp), axis=1)
            line[0][4+j].set_data(Orczyk[0:2,:])
            line[0][4+j].set_3d_properties(Orczyk[2,:])
        return line[0]
    if pid_signal_array_idx%3==1:
        line[1][0].set_data(t, p_pid_component_array)
        line[1][1].set_data(t, d_pid_component_array)
        line[1][2].set_data(t, error_pid_array)
        return line[1]
    if pid_signal_array_idx%3==2:
        line[2][0].set_offsets(points[0:42+trajectory_array_size,:])
        return line[2]


def main():
    suppress_qt_warnings()
    ani = animation.FuncAnimation(fig,
        animate,
        interval=1,
        blit=True)
    plt.show()


all_ports = serial.tools.list_ports.comports()
Arduino_Port = ""

ser = None
for port, desc, hwid in sorted(all_ports):
    if "Arduino" in desc:
        ser = serial.Serial(port, 57600, timeout=.1)

start = time.time()
main()