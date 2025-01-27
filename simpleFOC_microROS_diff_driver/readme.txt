
este controlador foi implementado para se comunicar via serial, aceitando comandos no formato:

LF1.25;			//define velocidade do motor Left em 1.25ms/s Forward(frente)
LB1.23;			//define velocidade motor Left em -1.23ms Backward(ré)
LV;			//pede velocidade do motor Left
LA;			//pede angulo do motor Left

RF
RB
RV
RA

OBS: caso seja enviado algum dos comandos:
LF; LB; RF; RB;
sem o numero, o esp32 vai entender como velocidade zero 0.0m/s 