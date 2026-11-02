# 纯底盘八段闭环测试

目的：开局扫描障碍→计算8段→整包上传STM32→依次走完→各站减速停留→回到实际起点。不进行扫码、识色、抓放或夹爪操作。用户明确允许此测试模式用定时停留代替站内作业。

## 已实现的流程

入口 `python3 -m tools.electrical_route_test`，复用现有雷达服务的HTTP接口，不重复打开雷达。先按显式区号配置，等待至少3个新的扫描序号且跨过当前证据累计窗口，再请求一次 `plan_mission`；即使网页已有READY计划，也重新生成本次计划。无路、过期、服务重启或配置变化均不上传。

路线以同一V1整帧上传，最多64点；当前使用新版 `car_code1 2` 按键协议：`--wait-key` 接收KEY1→区1、KEY0→区2，先ACK再开始本次扫描规划；换区使旧结果作废，0x16绑定request_id上传。`--start-zone 1/2`仅留作离线/不运动检查，真实执行要求按键入口。初始化仍需要实际车体参考点的地图X/Y和车头方向，不能拿雷达中心位姿顶替。

测试保持固定车头，`heading_deg`只能为−180/−90/0/90/180，且初始车头与之相同；利用麦轮前后/左右沿地图轴向道路走，不把动画方向当车头。真实起点与名义启停接近点不同的部分，拆成显式轴向连接点，末段反向连接回实际起点。连接点与路线均核对占据格、静态禁行区和现有车体余量；现有扫描漏检与静止定位误差仍不由此消除。

各站末点work_id=101…108。默认在长于接近距离的末端路段插入接近点：先按巡航RPM到接近点，停稳后用较低RPM走最后一段。短段直接低速到站。STM32的速度控制会按实际剩余距离继续降低速度。

**电控已有的闭环：** `route.c → chassis_motion.c → chassis_heading.c`。距离来自轮编码器、航向来自IMU；默认比例控制加RPM斜坡约束，进入位置/航向容差且四轮反馈在约±1 rpm内、指令近零，持续250ms才置底盘DONE；路线层随后进入WAIT_WORK。不是树莓派睡眠几秒后假装到达。

树莓派只把匹配路线号/点数/点索引/work_id且估计位置在目标附近的WAIT_WORK作为停车点。默认继续轮询2秒，再发明确标记的模拟WORK_DONE。8个站必须按顺序全部释放，且最终同一路线DONE、index=count、末点估计位姿符合要求，才报告测试done。事件可丢，流程靠匹配的STATUS ACK恢复。

**当前电控每个下发航点都会停稳**。树莓派默认 `stop_strategy=controller_deceleration`：不再插入作业点前120mm的额外接近点；同方向、同航向、同速度的直线中间点合并，保留所有作业点、拐点、掉头折返和速度变化点。不切角、不改变绕障轨迹。站点用巡航RPM上限交给电控已有剩余距离闭环降速到零，短距起步/回家连接仍用approach_rpm限速。剩余拐点仍停车，不是连续曲线巡航。

V1没有单独的“接近速度/减速起点”字段；`approach_distance_mm`仅在显式 `stop_strategy=split_approach` 的旧版对比模式使用。默认模式的 `approach_rpm` 是首尾短连接段上限，不能将其描述为最后120mm实测恒速。两种模式均保留8个WAIT_WORK及真实闭环停稳，测试驻留仍为2秒。站点名仅表示位置，不执行扫码或抓放业务。

## 文件

- `tools/electrical_route_test.py`：实时扫描、规划、上传、执行监控、站点停留及失败报告。
- `pi_vision/electrical_routes.py::compile_chassis_test`：测试路线、直线航点合并、首尾连接和固定航向。
- `tools/chassis_route_benchmark.py`：用保存快照和真实C控制器离线比较修改前后的点数/耗时，模拟结果不是实车预测。
- `config/chassis_test_profiles.example.json`：示例参数，实际起点留空，不能直接运行实车。
- `tools/chassis_route_sim.py`、`tools/chassis_route_host.c`：临时编译对方真实路线/底盘/IMU源码，模拟CAN、UART、电机和场地输入。
- `tests/test_chassis_route_test.py`：路线、作业停点、路径保真、控制流程及异常测试；其它V1测试仍保留。

测试路线保留 `valid_for_motion=false` 以表明未完成正式导航验收。只有独立测试入口的 `--execute --test-only-route` 才允许按测试标签上传；通用upload默认继续拒绝该路线。没有改动下载目录里的固件，没有烧录或启动真实运动。

## 现在可运行：完整软件模拟

**Mac终端：**

```bash
cd /Users/jzxzhou/Documents/ChatGPT/车子
python3 -m tools.chassis_route_sim '/Users/jzxzhou/Downloads/car_code1 2' \
  --zone 1 --heading 90 --key-mode --output output/chassis_route_test/photo_zone1.json
```

它走的是本项目V1编解码、真实 `route.c/chassis_motion.c/chassis_heading.c/imu.c`；用对方现有CAN/轮子/IMU模拟模型产生闭环反馈，不打开任何硬件。新版固件加`--key-mode`模拟按键请求/ACK/绑定上传；替换 `--zone 2 --heading -90` 测另一区（该角度只用于模拟）；加 `--fault imu`、`--fault stall` 或 `--fault stop` 注入IMU掉线、堵转或外部停止。故障例应返回测试failed，并核对控制器停止/故障状态；模拟断言通过时进程仍返回0。

## 接线后的准备

用户选择USB转TTL：TX接PC11、RX接PC10、GND共地、3.3V逻辑。Pi的CP2102/ttyUSB0是雷达，须确认新增电控适配器稳定路径；USB方案不需要改ttyAMA0控制台。先用electrical_comm watch-areas确认两键，退出释放串口再跑路线。不要直接把示例中的设备占位词粘贴执行。

本测试只针对所给固件的空 `Route_OnWork` 钩子。若电控以后在钩子里增加了机械臂动作，需选用底盘测试固件/禁用该动作后再跑；树莓派“不发VISION”不会自动禁用固件自己的钩子。

**树莓派终端，第1步：** 在工程中复制示例为本机配置：

```bash
cd /home/cuiguo/competition_vision
cp config/chassis_test_profiles.example.json config/chassis_test_profiles.json
```

两组配置必须保留 `coordinate_frame: "field_photo_ccw"`，实际车体起点 `x_mm/y_mm/yaw_deg` 均按照片同向场地填写：左下原点、右+X、上+Y、逆时针为正。区1沿通道朝二维码区的yaw/heading已设+90°；两区X/Y以及区2朝向仍待填写。不要复制雷达pose、旧地图坐标或IMU上电0°。旧配置须整套转换X/Y/yaw，不能仅补frame标签。两区都补全后才能进入按键测试。

编译器将原路网转换到照片同向坐标，再与上述实车起点连接；碰撞检查反变换到原地图。网页显示旋转开关不参与下发。详见ELECTRICAL_COMMUNICATION.md的统一坐标章节。示例轮转速20/15/8 rpm与接近距离120mm是测试初值；实际配置和报告继续Git忽略。

**树莓派终端，第2步：** 复用已按该摆放配置好的唯一雷达服务，先按键选区并规划复核，只确认按键、不上传运动路线：

```bash
python3 -m tools.electrical_route_test \
  --wait-key --port /dev/serial/by-id/实际电控适配器 \
  --profile config/chassis_test_profiles.json \
  --route-id 1 --output output/chassis_route_test/prepared.json
```

先启动程序再短按KEY1/KEY0并松开；此命令确认区号后会等新扫描、重新规划，并保存路线、启动快照、UPLOAD帧hex及从二进制解码的direction_review（每点先转/前后/左右动作）；执行模式也在START前打印并保存此记录。`--state-json`可改为保存快照做离线检查，但离线快照不能与`--execute`一起使用。规划后静止摆放发生变化，需要重新检查配置并重新生成。

**树莓派终端，第3步：** 接线/端口核对、起点确认且准备实车测试后，运行（将占位设备换成实际电控UART）：

```bash
python3 -m tools.electrical_route_test \
  --wait-key --port /dev/serial/by-id/实际电控适配器 \
  --profile config/chassis_test_profiles.json \
  --route-id 1 --station-dwell 2 \
  --execute --test-only-route \
  --output output/chassis_route_test/run.json
```

执行命令会重新扫描和规划，不执行上次prepared.json。route_id必须大于STM32当前路线号，同次上电第2轮可改为2，以此类推；旧id不自动重启。测试拒绝接管已RUNNING/WAIT_WORK的控制器。当前进程的Ctrl+C/SIGTERM、超时、故障、跳站、错误位姿都会尝试STOP并查询最终状态；报告保留失败原因及是否收到控制器停止确认。

## 证据及限制

2026-11-02 Mac：全套 `346 passed, 172 subtests passed`；新版V1与对方route.c/route_uart.c/route_keys.c的20项C主机测试通过。完整控制链模拟：区1/区2各25航点、24个实际控制器子动作、8次停站后DONE；均观察到速度下降，WAIT_WORK时模拟四轮转速在停稳范围且settled_ms≥250。IMU掉线、堵转、外部STOP三个例子均停止推进，不释放工作点。另有Python断连/超时/取消/旧计划/跳点/提前DONE等检查。

模拟报告在 `output/chassis_route_test/`；约211秒是模型中的时钟，不是本车实测耗时。约3mm的模型回程残差不是本车定位精度。部署结果见PROJECT_CONTEXT.md；真实UART/底盘验收仍待做。

运行后固定的是**开局障碍快照**。运动中不把静止雷达pose继续投影得到的占据格当真实障碍，也没有自动重规划或新增动态障碍急停。当前固件UART断连不会自动停车：Pi发送STOP若链路断开可能到不了，报告会标停止未确认；需保留电控本地取消/急停，后续再补链路看门狗。这些边界不阻塞软件流程测试，但不能据模拟通过宣称整车避障或比赛级稳定。

按键扩展与通用正式入口详见ELECTRICAL_COMMUNICATION.md。正式入口不会使用测试的定时WORK_DONE；夹爪未完成不影响本测试模式。两区实际车体起点配置暂留空，不伪造标定。

部署已完成：Pi `/home/cuiguo/competition_vision`，26文件SHA256一致，Pi345 passed/1 skipped；备份`backups/electrical_keys_20261102_28vEA6`。软件和示例配置已就位，未开真实UART或发运动。

2026-11-02照片同向坐标更新：68项相关Python测试通过；电控真实C的18种方向/转向顺序用例通过。区1按+90°进行按键→8站闭环模拟，区2用−90°模拟验证坐标转换（不是区2实车朝向确认）。初始X/Y、最终点、二进制航向、回传frame、旧frame拒绝及原地图障碍检查均有测试；实车方向/位置仍待短距验收。

## 按实测矩形检查场地边界（2026-11-02）

测试profile可增加`footprint`，四个方向距离均以电控定位参考点为起点，单位mm：front_mm（前）、rear_mm（后）、left_mm（左）、right_mm（右），另有margin_mm定位/尺寸余量。须包括雷达延长板等突出部件，不能仅填底盘尺寸。字段须全部为有限数值；未提供footprint时保留原圆形检查。

有footprint时，按固定车头旋转矩形检查每个目标位姿与起点，直线平移段的角点在端点之间，因而端点界限覆盖全段。若航向变化则拒绝，此检查不能替代转向扫掠校验。仅替换场地边界的圆形近似；静态禁区保留原中心线避障半径，并增加完整矩形外廓加余量的扫掠校验；不使用外接圆替代矩形而误报窄通道。不会把障碍余量缩小至半车宽。

用户确认前后长286mm、原底盘宽257mm；右侧雷达板位于前后中部，总宽294mm，因此右向伸出37mm，完整外廓已写入本轮实车配置。单看底盘，300方形启停区居中后两组边距21.5/7mm，此数值不包括板或任何定位余量。用户确认起点区1中央，之后实际配置可使用(150,150,90)，该点尚需完整外廓检查。

2026-11-02实车首段：完整外廓为前后286mm、原底盘宽257mm、右侧延长37mm（总宽294mm）。first_segment_only裁剪为首段4点，完成1次停站后DONE；终点电控估计(345,1197)mm、91.37°，无故障上报。仅编码器/IMU估计，实车方向/绝对位置仍待用户现场核对。车辆已不在初始区，禁止沿用起点重跑。原雷达静止预览尚不提供移动定位。

单段测试编译采用stage_limit=1并采集first_stage_only=True，只核验实际将执行的首段；即使八段批次整体因后续段不可达而标记no_path，首段仍须有新鲜path且所有首段几何与障碍检查通过。默认完整8段流程不受此选项影响。第二次实车重跑已DONE，回传估计(346,1197)mm、90.08°，两轮方向/坐标证据见PROJECT_CONTEXT.md。

2026-11-02完整8段实车记录：用户确认重新摆回区1中央、已处理前一轮电机报警后，明确要求复用原28航点且不重扫。巡航60rpm、接近15rpm、每站2秒测试停留，8站均完成并最终DONE、无故障，耗时约246.5秒；回传估计(153,153)mm/yaw88.75。估计误差不等于外部测量精度。原30/10rpm轮在完成2站后报地址2的CHASSIS_MOTOR_ERROR并STOP；未隐去该失败记录。两轮证据与复用快照的授权见PROJECT_CONTEXT.md。

2026-11-02区2实车：用户确认区2中央、车头仍+90°，本轮采用(150,2250,90)，巡航90/接近15rpm，新扫描生成28点路线，8站均完成、DONE、无故障。实车约236.35秒，最终估计(153,2251)mm、88.88°。上电准备期间一次MCU重启导致旧Client退出，未启动；新Client重新确认KEY0后才执行。全部TX（含STATUS）及RX原始分片已记录，和早期仅应用层解码的日志范围不同。
