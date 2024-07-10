cmd_/home/lemona16/tdrv009/Module.symvers := sed 's/\.ko$$/\.o/' /home/lemona16/tdrv009/modules.order | scripts/mod/modpost -m -a  -o /home/lemona16/tdrv009/Module.symvers -e -i Module.symvers   -T -
