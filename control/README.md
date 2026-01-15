# Controle lateral e longitudinal do carro no FSDS

## Running & Launching

```bash
    ros2 run control control_node.py
   ```

```bash
    ros2 launch control control.launch.py
   ```

## Overview
Este repositório contém a implementação do sistema de **controle de um carro autônomo**, responsável por garantir o correto seguimento de trajetória e a manutenção da velocidade desejada. O controle do veículo é organizado em três principais módulos:

- **Controle Longitudinal** (velocidade)
- **Controle Lateral de Alto Nível** (seguimento de trajetória)
- **Controle Lateral de Baixo Nível** (atuador de direção)

### Controlador longitudinal:

É responsável por garantir que o veículo alcance a velocidade desejada e mantenha essa velocidade ao longo do tempo, mesmo diante de perturbações. Sua estratégia de controle é realizada por meio de um **controlador PID (Proporcional–Integral–Derivativo)**, que atua sobre o erro entre a velocidade desejada e a velocidade atual do veículo.

### Controlador lateral de alto nível:

É responsável por receber o caminho calculado pelo módulo de path planning e determinar o ângulo de esterçamento desejado para que o veículo siga corretamente esse caminho e se mantenha dentro da trajetória desejada. Sua estratégia de controle utiliza o KLS (Kinematic Lateral Steering), que calcula o ângulo de direção ideal com base em erros geométricos, como: erro lateral em relação à trajetória e erro de orientação do veículo, com isso o valor de esterçamento calculado é então enviado para o controle lateral de baixo nível.

### Controlador lateral de baixo nível:

Atua diretamente sobre o **atuador da direção, este r**ecebe o valor de esterçamento desejado e manda um sinal de PWM, garantindo que esse ângulo seja efetivamente atingido pelas rodas do veículo. Esse controle também é realizado por meio de um **controlador PID**, que atua sobre o erro entre: ângulo de esterçamento desejado e o ângulo de esterçamento real medido no sistema.
