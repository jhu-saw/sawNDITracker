## Aurora hardware versions

Aurora component versions need to partially match, i.e. one can't always use older components on a newer system.

This document contains empirical results.  In doubt, reach out to NDi.  

The following might help users putting a system together using components from different origins.

### Serial numbers and versions

Field Generator (FG)
* V1: Serial No. FGb4-Sxxxxx
* V2: Serial No. FGc0-Sxxxxx

Control Units (SCU)
* V1: Serial No. SCa4-Sxxxxx   
* V2: Serial No. SCb2-Sxxxxx

Sensor Interface Units (SIU)

### Compatibility

| SCU\SIU          | V1  | V2  | V3  |
|:----------------:|:---:|:---:|:---:|
| V1 (SCa4-Sxxxxx) | Yes | Yes | No  |
| V2 (SCb2-Sxxxxx) | Yes | Yes | Yes |
| V3               | Yes | Yes | Yes |

| SCU\FG           | V1 (FGb4-Sxxxxx) | V2 (FGc0-Sxxxxx) | V2 Table-Top (FGd0-Sxxxx) | 
|:----------------:|:----------------:|:----------------:|:-------------------------:|
| V1 (SCa4-Sxxxxx) | Yes              | No               | No                        |
| V2 (SCb2-Sxxxxx) | No               | Yes              | Yes                       |
| V3 (SCd4-Sxxxxx) | No               | Yes              | Yes                       |
