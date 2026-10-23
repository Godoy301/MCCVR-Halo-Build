"""Generate independent ODST topology expectations from the inspected user map."""
import argparse
import json
from pathlib import Path

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--source-root',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
evidence=json.loads((a.source_root/'docs/ODST-MODDED-WEAPONS-MAP-EVIDENCE.json').read_text())
lines=['// Generated asset expectations; production code is included directly.',
       'struct Asset { uint32_t checksum; int count,bodyCount,modelCount; std::array<int32_t,64> body,model,parents; };',
       'static constexpr Asset assets[]{']
for weapon in evidence['weapons']:
    for fp in weapon['first_person']:
        graph=fp['graph']
        byid={n['string_id']:n['index'] for n in graph}
        assert graph[-1]['name']=='camera_control'
        for body in evidence['bodies']:
            if not all(n['string_id'] in byid for n in body['nodes']):continue
            # A human graph contains most Elite names too. Require the body's
            # full prefix topology in the asset fixture, not a name subset.
            bodymap=[byid[n['string_id']] for n in body['nodes']]
            if bodymap!=list(range(len(bodymap))):continue
            parents=[n['parent'] for n in graph]
            assert parents[:len(bodymap)]==[n['parent'] for n in body['nodes']]
            model=[byid[n['string_id']] for n in fp['model_nodes']]
            assert not set(bodymap)&set(model)
            values=[body['checksum'],len(graph),len(bodymap),len(model)]
            lines.append('{'+','.join(map(str,values))+',{' + ','.join(map(str,bodymap))+'},{'+','.join(map(str,model))+'},{'+','.join(map(str,parents))+'}},')
lines.append('};')
a.output.parent.mkdir(parents=True,exist_ok=True)
a.output.write_text('\n'.join(lines)+'\n')
