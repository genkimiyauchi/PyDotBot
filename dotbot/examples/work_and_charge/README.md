# DotBot Experiment Example: Work and Charge

This experiment makes each robot continuously move between a virtual work region and charge region. The example is based on the work from this [repo](https://github.com/openswarm-eu/swarm-energy-replenishment).

---

## 1. Start the simulator

First, start the DotBot controller in **simulator mode** with the correct configuration:

```bash
python -m dotbot.controller_app --config-path config_sample.toml -p dotbot-simulator -a dotbot-simulator --log-level error
```

## 2. Run the experiments

Run this experiment:

```bash
python -m dotbot.examples.work_and_charge.work_and_charge
```

---

- The ```models/``` directory contains the control logic ```supervisor.yaml```, which is used by ```work_and_charge.py```. Other model and script files are used to design the control logic in [Nadzoru 2](https://github.com/openswarm-eu/Nadzoru2).