# Projeto Teach and Repeat - Contexto e Objetivos

## Descrição do Projeto
Projeto de TCC para aprimorar um local planner baseado em curvas de Bézier, inspirado nos planners do NAV2 stack, com foco em navegação logística incluindo reboques.

## Objetivos Finais
- Local planner que considere a cinemática do robô e reboques
- Navegação logística com até 2 reboques
- Integração futura com o pacote NAV2
- Suporte para robôs differential drive e Ackermann
- Footprint para trailers baseado nos movimentos do rebocador
- Desvio de obstáculos considerando cinemática completa

## Problemas Identificados (Situação Atual)

### 1. Velocidade Constante ⚠️ CRÍTICO
- Velocidade linear fixa em 0.2 m/s
- Desempenho limitado, especialmente em curvas
- **Solução proposta**: Velocidade adaptativa baseada na curvatura do segmento à frente

### 2. Problema de Waypoint Following ⚠️ CRÍTICO  
- Robô só avança para próximo waypoint quando dentro de `threshold_dist = 0.8m`
- Se passar do ponto mas não estiver próximo o suficiente, tenta voltar
- **Comportamento desejado**: Continuar em frente se estiver próximo, mesmo que não exatamente no ponto

### 3. Atraso em Curvas ⚠️ CRÍTICO
- Sistema demora para reagir em curvas acentuadas
- Parece "sempre atrasado" nas manobras
- **Causa provável**: Lookahead fixo inadequado para diferentes tipos de trajetória

### 4. Lookahead Fixo (EM DESENVOLVIMENTO) 🔄
- Atualmente: 15 pontos × 0.05m = 0.75m fixo
- **Problema**: Inadequado tanto para retas quanto para curvas
- **Solução em implementação**: Lookahead adaptativo baseado em curvatura

### 5. Função de Custo Simples
- Apenas distância euclidiana com peso decrescente
- Não considera orientação, suavidade, ou estabilidade
- **Melhoria futura**: Função multi-objetivo

### 6. Modelo Cinemático Simplificado
- Integração simples para lookahead paths
- Não considera adequadamente limitações cinemáticas
- **Melhoria futura**: Modelo mais preciso para differential drive/Ackermann

## Contexto Técnico

### Ambiente de Operação
- **Local**: Ambientes internos de logística
- **Obstáculos**: Presença de obstáculos dinâmicos
- **Estrutura**: Ambiente estruturado

### Especificações do Sistema
- **Rebocador**: Differential drive OU Ackermann
- **Reboques**: Máximo 2 reboques
- **Sensores**: Apenas no rebocador (reboques sem sensores)
- **Localização**: Posição dos trailers calculada com base nos movimentos do rebocador

### Parâmetros Atuais
```python
# Robot params
max_vel_x = 0.2          # m/s - MUITO BAIXO
max_vel_theta = 1.0      # rad/s
threshold_dist = 0.8     # m - distância para considerar waypoint atingido

# Lookahead params (FIXOS - PROBLEMA)
points_per_paths = 15    # pontos
dist_btw_points = 0.05   # m
lookahead_total_paths = 50  # quantidade de trajetórias candidatas

# Simulation
sim_steps = 30           # sub-passos para integração
```

## Estratégia de Desenvolvimento (Faseada)

### 🔄 FASE 1: Lookahead Adaptativo (EM ANDAMENTO)
**Objetivo**: Resolver o "atraso em curvas"

**Implementação**:
- Lookahead baseado em **distância** (metros) ao invés de quantidade de pontos
- Distância adaptativa baseada na curvatura local
- Curvatura calculada usando fração da distância do lookahead (40%)

**Parâmetros Propostos**:
```python
base_lookahead_distance = 1.2      # metros - distância base
curvature_analysis_fraction = 0.4  # 40% da distância do lookahead
min_lookahead_distance = 0.6       # metros - mínimo em curvas
max_lookahead_distance = 2.0       # metros - máximo em retas
```

**Lógica**:
- Curvas acentuadas → lookahead menor → reação mais rápida
- Trechos retos → lookahead maior → melhor antecipação

### 📋 FASE 2: Controle de Velocidade Adaptativo
**Objetivo**: Resolver velocidade constante inadequada

**Implementação**:
- Velocidade baseada na curvatura do segmento à frente
- Fórmula: `v_max = base_speed / (1 + k * curvature)`
- Suavização de transições de velocidade

### 📋 FASE 3: Waypoint Following Melhorado
**Objetivo**: Evitar comportamento de "voltar atrás"

**Implementação**:
- Critério híbrido: distância + progresso no path
- Tolerância de "overshoot" permitida
- Avanço baseado em progresso relativo

### 📋 FASE 4: Função de Custo Multi-objetivo
**Componentes propostos**:
- Desvio de posição
- Desvio de orientação  
- Suavidade (mudanças de curvatura)
- Velocidade de convergência
- Estabilidade (comandos angulares)

### 📋 FASE 5: Modelo Cinemático Modular
**Objetivo**: Preparar para reboques
- Suporte differential drive e Ackermann
- Estrutura modular para múltiplos veículos
- Predição de posição dos trailers

## Configuração Atual do Sistema

### Arquivos Principais
- `repeat_bezier_path.py` - Nó principal do planner
- `config.yaml` - Configurações (para uso futuro)
- `compare_bezier_lookahead.py` - Função de custo atual
- `utils.py` - Funções auxiliares

### Dependências ROS2
- `geometry_msgs` - Twist, Point, PoseStamped
- `visualization_msgs` - Markers para debugging
- `nav_msgs` - Path (futuro)

## Decisões Técnicas Tomadas

### ✅ Lookahead Adaptativo
- **Método escolhido**: Variar distância baseada em curvatura
- **Curvatura**: Calculada em fração da distância do lookahead (40%)
- **Responsividade**: Priorizada sobre estabilidade para curvas

### ✅ Obstáculos
- **Decisão**: Não implementar verificação de obstáculos inicialmente
- **Justificativa**: Focar primeiro na cinemática e seguimento de path

### ✅ Parametrização
- **Decisão**: Migrar para config.yaml posteriormente
- **Atual**: Manter hardcoded durante desenvolvimento para agilidade

### ✅ Integração NAV2
- **Decisão**: Não priorizar agora
- **Justificativa**: Melhor ter código funcional antes de adaptar para plugin

## Status Atual
- **Problema identificado**: Lookahead fixo causando atraso em curvas
- **Solução em desenvolvimento**: Lookahead adaptativo por distância
- **Próximo passo**: Implementar cálculo de curvatura e adaptação dinâmica
- **Teste pendente**: Validação do comportamento em curvas vs. retas
