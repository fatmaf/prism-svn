3. My code for UCT and BRTDP is here: https://github.com/bfalacerda/prism/tree/devel

Important files are in prism/prism/src/explicit/ and are BRTDP.java, BRTDPModelChecker.java, MCTSModelChecker.java and UCT.java.

To run you can do

bin/prism tests/mcts/bounded/clocked_coffee.prism tests/mcts/bounded/clocked_coffee.prop -mcts

or

bin/prism tests/mcts/bounded/clocked_coffee.prism tests/mcts/bounded/clocked_coffee.prop -brtdp
