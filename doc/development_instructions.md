# Instructions para Desenvolvimento - Teach and Repeat Planner

## Princípios de Desenvolvimento

### 🎯 Metodologia
- **Desenvolvimento incremental**: Uma funcionalidade por vez
- **Validação contínua**: Testar cada melhoria antes de prosseguir
- **Discussão prévia**: Explicar bem antes de implementar
- **Pequenos passos**: Evitar mudanças grandes de uma vez

### 💬 Estilo de Comunicação
- **Conversação técnica**: Discutir soluções em detalhes
- **Sugestões ativas**: Propor melhorias e identificar problemas
- **Explicação didática**: Explicar bem as implementações
- **Questionamento**: Fazer perguntas para entender melhor o contexto

## Guidelines Técnicas

### 📁 Estrutura de Arquivos
- **Manter organização atual**: Não reorganizar estrutura sem discussão
- **Scripts auxiliares**: Usar pasta `scripts/` para funções utilitárias
- **Documentação**: Criar/manter docs na pasta `docs/`
- **Configuração**: Migrar gradualmente para `config.yaml`

### 🔧 Padrões de Código
- **ROS2 Python**: Seguir padrões do ROS2
- **Nomenclatura**: 
  - Gradualmente migrar de "tractor" para "robot"
  - Manter compatibilidade durante transição
- **Parâmetros**: Eventualmente migrar hardcoded para ROS parameters
- **Comentários**: Manter em português (preferência do desenvolvedor)

### 🧪 Implementação
- **Validação prévia**: Sempre explicar mudanças antes de implementar
- **Backup implícito**: Git para controle de versão
- **Testes incrementais**: Implementar, testar, validar, prosseguir
- **Debug visual**: Manter markers do RViz para visualização

## Contexto do Problema Atual

### 🎯 Foco Atual: Lookahead Adaptativo
**Problema**: Sistema "atrasado" em curvas devido a lookahead fixo inadequado

**Solução em desenvolvimento**:
1. Lookahead baseado em distância (metros) ao invés de pontos
2. Adaptação dinâmica baseada na curvatura local
3. Curvatura calculada em fração da distância do lookahead

### 📊 Abordagem Técnica
- **Curvatura**: Usar distância ao invés de quantidade de pontos fixa
- **Responsividade**: Priorizar reação rápida em curvas
- **Antecipação**: Manter visão maior em trechos retos

## Regras de Implementação

### ✅ O Que Fazer
- Explicar implementação antes de codificar
- Propor parâmetros iniciais para discussão
- Manter funcionalidade existente durante transição
- Usar markers RViz para debug visual
- Discutir trade-offs de cada solução

### ❌ O Que Evitar
- Mudanças grandes sem discussão prévia
- Quebrar funcionalidade existente
- Implementar múltiplas features simultaneamente
- Assumir parâmetros sem validação
- Reorganizar estrutura sem necessidade

## Prioridades Técnicas

### 🔥 Alta Prioridade
1. **Lookahead Adaptativo** - Em desenvolvimento
2. **Controle de Velocidade** - Próxima fase
3. **Waypoint Following** - Após velocidade

### 📋 Média Prioridade
4. **Função de Custo** - Após itens críticos
5. **Modelo Cinemático** - Preparação para reboques
6. **ROS Parameters** - Refatoração gradual

### 📝 Baixa Prioridade
7. **Integração NAV2** - Futuro distante
8. **Obstáculos** - Não implementar agora
9. **Documentação Doxygen** - Após funcionalidade

## Fluxo de Trabalho

### 1. 💡 Discussão da Solução
- Apresentar problema específico
- Propor soluções alternativas
- Discutir prós/contras
- Escolher abordagem

### 2. 📋 Planejamento da Implementação
- Definir parâmetros iniciais
- Identificar funções a modificar
- Planejar testes de validação
- Considerar impactos

### 3. 🔧 Implementação
- Mostrar código específico a modificar
- Explicar mudanças linha por linha
- Implementar com aprovação
- Manter debug visual

### 4. 🧪 Validação
- Testar funcionalidade
- Verificar comportamento visual (RViz)
- Ajustar parâmetros se necessário
- Validar melhoria real

### 5. ➡️ Próxima Iteração
- Documentar resultado
- Identificar próximo problema
- Reiniciar ciclo

## Comunicação Efetiva

### 📞 Perguntas Importantes
- "O que você acha desses valores iniciais?"
- "Prefere abordagem A ou B?"
- "Quer que eu implemente isso diretamente?"
- "Como você quer testar isso?"

### 🎯 Objetivos de Discussão
- Entender requisitos específicos
- Validar abordagem técnica
- Definir parâmetros apropriados
- Planejar testes adequados

### 💭 Considerações Futuras
- Sempre mencionar impacto em reboques
- Considerar escalabilidade
- Pensar em integração NAV2
- Manter flexibilidade para diferentes robôs

## Estado Atual da Conversa

### 🔄 Em Desenvolvimento
**Lookahead Adaptativo**: 
- Implementação de distância ao invés de pontos
- Cálculo de curvatura em fração da distância
- Adaptação dinâmica baseada em curvatura local

### ⏳ Próximos Passos
1. Finalizar implementação do lookahead adaptativo
2. Testar e validar comportamento
3. Ajustar parâmetros conforme necessário
4. Prosseguir para controle de velocidade

### 📝 Decisões Pendentes
- Valores exatos dos parâmetros iniciais
- Método específico de cálculo de curvatura
- Estratégia de teste e validação
