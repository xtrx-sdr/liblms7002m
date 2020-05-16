########################################################################
## Parse the LMS7002M register map (text dump from pdf)
## and create usable C structures and enum definitions
########################################################################

import sys
import os
import re
import json
import functools
from Cheetah.Template import Template

def get_name(reg, field_name, category):
    #TODO optional reg[name]
    reg_name = reg['addr'].upper()
    field = reg['fields'][field_name]
    return 'LMS7002_' + category.upper() + '_' + field_name.upper()
    #return 'LMS7002_' + field_name.upper()

def get_name_off(reg, field_name, category):
    return get_name(reg, field_name, category) + "_OFF";

def get_name_msk(reg, field_name, category):
    return get_name(reg, field_name, category) + "_MSK";

def get_default(reg):
    if 'default' in reg:
        return hex(eval('0b'+reg['default'].replace(' ', '')))
    return 0

def get_options(reg, field_name):
    field = reg['fields'][field_name]
    if 'options' in field:
        return [(('%s_%s'%(get_name(reg, field_name, "OPT"), k)).upper(), v) for k,v in sorted(field['options'].items(), key=lambda x:x[1])]
    return None

def get_shift_mask(reg, field_name):
    field = reg['fields'][field_name]
    bits = field['bits']
    if ':' in bits:
        low, high = sorted(map(int, bits.split(':')))
        assert(low >= 0)
        assert(high < 16)
        mask = hex(eval('0b'+('1'*(high-low+1))))
        return low, mask
    return int(bits), "0x1"

def sorted_field_keys(reg):
    return reversed(sorted(reg['fields'].keys(), key=lambda x: list(map(int, reg['fields'][x]['bits'].split(':')))[0]))

def sorted_field_keys_str(reg):
    return functools.reduce(lambda x, y: str(x) + ", " + str(y), sorted_field_keys(reg))

HEAD="""
/* LMS7002 definitions for HOST/MCU operations */
#ifndef LMS7002M_DEFS_H
#define LMS7002M_DEFS_H

/* Auto generated file, DO NOT MODIFY */
"""


TMPL="""
/*************************************
 * $category configuration registers
 *************************************/
#for $reg in $regs
enum lms7002m_reg_$reg.addr {
    #for $field_name in $sorted_field_keys($reg)
    #set $shift, $mask = $get_shift_mask($reg, $field_name)
    $get_name_off($reg, $field_name, $category) = $shift,
    #if $mask != "0x1"
    $get_name_msk($reg, $field_name, $category) = $mask,
    #end if
    #end for
};
\#define MAKE_LMS7002_${reg.addr}( $sorted_field_keys_str($reg) ) ( \\
#for $field_name in $sorted_field_keys($reg)
#set $shift, $mask = $get_shift_mask($reg, $field_name)
    (((${field_name}) & ${mask}) << ${shift}) | \\
#end for
    (${reg.addr}0000 | 0x80000000))
#for $field_name in $sorted_field_keys($reg)
#set $shift, $mask = $get_shift_mask($reg, $field_name)
\#define GET_LMS7002_${category}_${field_name}(r) (((r) >> $shift ) & $mask)
#end for

#end for
"""

ALLTMPL="""

#for $reg in $regs
#for $field_name in $sorted_field_keys($reg):
#if $get_options($reg, $field_name)
#for $k,$v in $get_options($reg, $field_name)
\#define $k ${v}u
#end for;
#end if
#end for
#end for

/* All known registers */
enum lms7002_regs {
#for $reg in $regs
    LMS7002M_$reg.addr = $reg.addr,
#end for
};

/* All indexed registers (to store whole state) */
enum lms7002_reg_idxs {
#set $idx = 0
#for $reg in $regs
    LMS7002M_IDX_$reg.addr = $idx,
#set $idx = $idx + 1
#end for
    LMS7002M_IDX_COUNT = $idx
};


#endif /* LMS7002M_DEFS_H */
"""

if __name__ == '__main__':
    print(HEAD)

    regs = list()
    for arg in sys.argv[1:]:
        if arg.endswith('defaults.json'):
            continue
        else:
            rg = json.loads(open(arg).read())
            rg = dict([(r['addr'].lower(), r) for r in rg])
            #sort registers back into list
            rg = sorted(rg.values(), key=lambda x: eval(x['addr']))

            regs.extend(rg)
            code = str(Template(TMPL, dict(
                regs=rg,
                category=os.path.basename(arg).split(".")[0].upper(),
                get_name_off=get_name_off,
                get_name_msk=get_name_msk,
                get_default=get_default,
                get_options=get_options,
                get_shift_mask=get_shift_mask,
                sorted_field_keys=sorted_field_keys,
                sorted_field_keys_str=sorted_field_keys_str,
            )))
            print(code)



    regs = dict([(r['addr'].lower(), r) for r in regs])
    #sort registers back into list
    regs= sorted(regs.values(), key=lambda x: eval(x['addr']))

    code = str(Template(ALLTMPL, dict(
        regs=regs,
        get_name_off=get_name_off,
        get_name_msk=get_name_msk,
        get_default=get_default,
        get_options=get_options,
        get_shift_mask=get_shift_mask,
        sorted_field_keys=sorted_field_keys,
        sorted_field_keys_str=sorted_field_keys_str,
    )))

    print(code)
