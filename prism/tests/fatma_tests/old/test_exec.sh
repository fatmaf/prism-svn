#!/bin/bash

fn=$1
mext=$2
prop=$3
comm="../../bin/prism $fn.$mext $prop.props -ex -exportadv ${fn}_adv.tra -exportprodstates ${fn}_prod.sta -exportstates $fn.sta -exportlabels $fn.lab -exportprodtrans ${fn}_prod.tra -exporttarget ${fn}_tar.lab -exporttransdot ${fn}_trans.dot -exporttransdotstates ${fn}_sta.dot"
echo $comm
eval $comm
