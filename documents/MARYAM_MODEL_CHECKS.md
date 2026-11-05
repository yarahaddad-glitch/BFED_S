# Maryam model: numerical checks

Internal consistency only; independent CAD and tutor approval remain to be recorded.

| Case | FK matrix maximum error | Jacobian maximum error | IK position error (mm) | IK orientation error (deg) |
| --- | --- | --- | --- | --- |
| 1 | 0 | 1.4e-10 | 1.836e-06 | 1.75e-07 |
| 2 | 0 | 1.37e-10 | 1.848e-10 | 4.151e-11 |
| 3 | 0 | 1.06e-10 | 7.851e-11 | 1.32e-11 |
| 4 | 0 | 1.45e-10 | 2.006e-12 | 3.263e-14 |

RMRC: 141 checked samples; final position error 0.206 mm.

The IK targets are generated from known FK configurations. This demonstrates solver consistency, not agreement with an independently measured robot. A different joint solution can reach the same pose.

The JSON companion contains target coordinates, joint configurations and IK solutions.

Reproduce from the assignment root:

```powershell
& '../.venv/Scripts/python.exe' -B -m Maryam_Work.verify_model
```

Still record: a labelled DH-frame sketch, independent mesh/CAD flange checks, CAD asset attribution, tutor/model approval, and the real-robot demonstration.
