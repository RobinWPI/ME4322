# Homework 1 — PMKS+ model / 六杆机构模型

## Open in a browser / 在浏览器中打开

**[Open the saved PMKS+ model / 打开已保存的模型](https://app.pmksplus.com/?3u.BG.N,0.2112.6A,A,Lu,7b,0.0B,B,Q6,FU,0.0C,C,3-,GB,0.4D,D,4T,t,0.0E,E,33,di,0.0F,F,0FK,eA,0.4G,G,o,38,0.0H,H,0Qp,12a,0..YRAB,AB,1zU,44,O1,Bb,B3E2CD,A,B,,.YRBC,BC,4lI,q_,F5,Fq,FDCDAC,B,C,,.YRDCE,DCE,8B8,4MI,3m,KO,CBD5E8,C,E,D,,.YREF,EF,3yW,Uo,066,dx,F4CAE4,E,F,,.YRGFH,GFH,EaW,ODY,0C-,Yp,E6F5C9,G,F,H,,..2F1,GFH,Q,0Qp,12a,0Qp,wm,mr0..N_Q)**

No separate website deployment is required: the URL contains the model and opens it in the PMKS+ web app. The Internet connection and PMKS+ service are required; this is not an offline copy of PMKS+.

不需要另外部署网站：链接包含模型数据，点击后直接在 PMKS+ 网页版中打开。需要联网且 PMKS+ 服务可用；本文件包不是 PMKS+ 离线程序。

- On Windows, download and double-click `Open_PMKS_Model.url`, or open the link above in Chrome.
- Alternatively, open [PMKS+](https://app.pmksplus.com/), select **Open**, and choose `model.pmks`.
- 在 Windows 下载并双击 `Open_PMKS_Model.url`，或将上方链接在 Chrome 中打开。
- 也可以在 PMKS+ 网页中选择 **Open**，导入 `model.pmks`。

## Restore the exact input speed / 恢复精确输入转速

After reopening, set **Settings → Input Speed = 23.1481481481481 RPM**, counterclockwise. The PMKS+ save format stores integer RPM, so the saved model initially opens at 23 RPM. This is a format limitation, not a change to the intended simulation speed.

重新打开后，将 **Settings → Input Speed 设为 23.1481481481481 RPM**，方向选择逆时针。PMKS+ 保存格式仅保留整数 RPM，因此保存模型打开时为 23 RPM；这是格式限制，不是更改设计转速。

Target: 12,500 parts in 9 hours; one input revolution per complete out-and-back cycle and part; period 2.592 s; constant input angular velocity 2.42406840554768 rad/s and zero input angular acceleration.

目标：9 小时 12,500 件，输入每转一圈完成一次往返及一个零件，周期 2.592 s；输入角速度恒定为 2.42406840554768 rad/s，输入角加速度为零。

## Model contents / 模型内容

- Links: AB, BC, DCE, EF, GFH; ground joints: A, D, G; input: AB at A.
- SI metres and radians; H lies beyond F on the GF extension.
- Q = 200 N downward at H on GFH: global components (0, -200) N. The force anchor moves with GFH, but its direction stays globally vertical.
- SW masses and centroidal inertias are included, with transformed global centers of mass.
- 连杆为 AB、BC、DCE、EF、GFH；固定点为 A、D、G；AB 在 A 处输入。
- 采用 SI 米制和弧度；H 位于 GF 越过 F 的延长线上。
- GFH 的 H 点施加向下 Q = 200 N，全局分量为 (0, -200) N；作用点随杆运动，方向始终全局竖直向下。
- 包含 SW 质量、质心转动惯量及变换后的全局质心坐标。

| Link / 连杆 | Mass / 质量 (kg) | Centroidal inertia / 质心惯量 (kg·m²) | Saved CoM / 保存质心 (m) |
|---|---:|---:|---|
| AB | 8.03 | 0.26 | (1.537, 0.741) |
| BC | 19.41 | 3.39 | (0.965, 1.012) |
| DCE | 33.48 | 17.81 | (0.240, 1.304) |
| EF | 16.16 | 1.97 | (-0.390, 2.555) |
| GFH | 59.68 | 99.17 | (-0.831, 2.227) |

## Precision and assumptions / 精度与假设

The PMKS+ codec saves coordinates to 0.001 m. H and the Q anchor coincide exactly at saved H = (-1.715, 4.260) m. `model.json` records both saved values and higher-precision calculation targets. To restore more precise CoMs, enter their target coordinates after all joint geometry edits; PMKS+ can recalculate CoMs when joints are edited.

PMKS+ 编码保存坐标至 0.001 m。H 与 Q 作用点在保存坐标 H = (-1.715, 4.260) m 处完全重合。`model.json` 同时记录保存数值和较高精度的计算目标。若需恢复质心目标坐标，应在所有关节几何编辑完成后录入；修改关节时 PMKS+ 可能重算质心。

SolidWorks screenshot values have only two displayed decimals. Use centroidal Lzz, not inertia about the CAD origin. The current material assumption is 6061-T6 (density 2700 kg/m³); density alone does not verify alloy grade or temper. The straight CAD DCE model differs slightly from the specified joint geometry: C is about 5.49 mm off line DE.

SW 截图仅显示两位小数，采用的是过质心的 Lzz，而不是过 CAD 原点的惯量。当前材料假设为 6061-T6（密度 2700 kg/m³）；密度本身不能验证牌号或热处理状态。CAD 中 DCE 为直杆，而给定关节 C 偏离 DE 约 5.49 mm，存在小幅模型差异。

Q is a prescribed constant external force. No payload mass has been added to GFH, and this model does not implement an artifact's acceleration-dependent inertial reaction. Link self-weight/gravity is a separate simulation setting; a downward Q alone does not enable gravity.

Q 是给定恒定外力。GFH 未叠加工件质量，本模型不包含工件随加速度变化的惯性反力。连杆自重/重力是独立的仿真设置；添加向下的 Q 不等同于开启重力。

## Files / 文件

- `model.pmks`: prepared PMKS+ import model / PMKS+ 导入模型。
- `Open_PMKS_Model.url`: browser shortcut / 浏览器快捷方式。
- `model.json`: portable, readable parameters and precision notes / 可读参数与精度说明。
- `README.md`: opening instructions / 打开说明。

This is a prepared model import bundle, not a native browser export or a hosted copy of the PMKS+ application.

这是准备好的模型导入文件包，不是浏览器原生导出，也不是另行托管的 PMKS+ 程序。
