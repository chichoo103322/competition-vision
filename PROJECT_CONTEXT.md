# 项目交接上下文

更新：2026-11-03。供全新聊天接续；先读 `AGENTS.md`。本文只记录最终决策、可复用命令与完成边界，不含凭据。

## 移除所有按键启动锁（2026-11-03，最新）

用户明确要求调试与正式入口都不做持久化自锁；每次按键前默认车辆已放在对应启停区，KEY1/KEY0选择区后从头进行新扫描、规划和运行。实现删除RoundLatch与`--reset-round`；每次完成/失败后都回待机，服务重启不重放旧按键；控制器未空闲或有故障只留在WAIT_CONTROLLER，恢复后自动READY。两种systemd服务均设为on-failure自动重启。比赛每轮允许几次仍需遵守对应赛规/裁判要求。

部署此改动前发生一次干扰用户调试的错误：上一轮测试服务已READY，用户新KEY1启动了route_id1793704491区1的16点路线。助手为了更新代码停止了运行中的服务，SIGTERM令runner取消并发送0x12 STOP；STM32回报STOPPED、fault0/chassis_fault0，停止时已到work_id101首站，估计位姿约(1188,1200)mm/+90°。这是助手导致的中断，不是电控自行停车。

无锁版本、两种systemd unit及说明已部署；改前代码、unit、round_latch、status、result和wire日志备份于 `backups/remove_locks_20261103/`。旧Pi `round_latch.json` 已移除。Mac 92项按键/通讯/路线测试通过，Pi语法、systemd verify和文件SHA256一致。

用户随后要求加快两区测试速度。Pi本机测试配置均改为move_rpm=120（schema上限）、approach_rpm=30、turn_rpm=15，配置校验通过、备份于 `backups/speed_adjust_20261103/chassis_button.local.json`，服务已重启加载。当前服务active但状态`WAIT_CONTROLLER`，没有收到STM32 STATUS回复；无round_latch文件，未发生新路线上传/START。正式比赛本机配置不存在，正式后端也未接通，不能声称比赛流程已可执行或已调速。用户要求不要监听后续按键；下一步仅按其明确请求响应。

## 按键测试待机常驻修复（2026-11-03，最新）

用户反馈独自测试按键时无反应。树莓派只读核对确认根因：上一轮区2成功后，测试服务按旧逻辑正常退出（systemd `Restart=no`），持久化 `round_latch.json` 仍存在；按键监听进程实际为inactive。此前每次聊天测试由助手手动复位本轮锁并重启服务，所以看起来只有对话时按键才有效。不是雷达、路径规划或电控未收到已发路线的问题。

修改测试入口：一轮成功后释放本轮锁，继续复用UART并回到READY，等待下一次READY后的新实体键；失败仍保留锁，不自动重跑。测试systemd对未处理进程故障按3秒间隔重启，若本轮已锁则不会重新发车。正式比赛入口的一次启动锁未变。新增成功测试可重入/正式和失败保持锁的测试；Mac相关60项通过。

修改脚本、unit和操作说明已SHA256核对部署到Pi，原脚本及unit备份于 `backups/button_repeatable_20261103/`。归档上轮DONE状态与日志、复位旧锁并启动服务后，现场状态为active/READY，STM32 STATUS通信正常。READY前一个旧KEY0请求（tick 40551，早于基线 tick 506900）已被正确ACK并忽略；新进程日志当前只有STATUS与AREA_ACK帧，没有路线上传或START，车辆未因本次部署启动。现在需要READY之后再短按并松开一次KEY0。服务重启及连续测试已部署验证；整机断电后自动启动仍未做断电实测。

## KEY0区2实车全程成功（2026-11-03，最新）

用户要求现在按KEY0测试。按新轮测试授权归档复位并恢复待机；READY前的KEY0 request1/tick18723被正确忽略，用户获通知后重新短按，收到新KEY0 request2/tick53636。两区既有本机配置无修改，区2初始(150,2250)mm/+90°，用户本轮按先前区2摆放约定测试。重新扫描得到16点，route_id1793703261绑定上传，START获接受。当前电控继续回phase5，Pi解析正常。

8站101..108依次WAIT_WORK/phase4、2秒测试驻留/模拟WORK_DONE，最终DONE/index16/count16、fault0/chassis_fault0。实际START ACK到首次DONE约122.91秒（约2分3秒，含测试驻留，不含真实扫码/抓放），最终电控估计(153,2250)mm/yaw89.86°，未做外部精度测量。STOP发送0条，全程无超时；区1和区2实体按键路径均有phase5修复后的成功实机记录。路线不同，不能把区1/区2时间直接当相同路径性能对比。

Mac忽略目录output/chassis_zone2_phase5_success_20261103保存本轮全部TX/RX_RAW、prepared_live/result/status/summary及区2_KEY0_完整日志.zip，Pi原日志仍在output/chassis_button。服务完成正常退出，一次锁保留，没有自动重跑；实时journal监看SSH已结束，独立雷达仍运行。下一次测试仍需显式重新准备与新键；比赛轮内不自动解除一次锁。此轮未改代码/速度、未Git提交。

## phase5修复后区1实车全程成功（2026-11-03，最新）

用户要求现在测试并实时看记录。只发STATUS确认MCU已重启EMPTY/无故障，旧服务inactive；按新一轮测试授权归档并复位本轮锁、启动服务。READY前积压的KEY1 request1/tick24142被正确忽略。待机期间随后观察到一次无回复/MCU重启，服务自动重建READY基线（tick1102）；接收此后真正新KEY1 request1/tick3371，ACK并采集新扫描。准备19点，route_id1793702946，上传、START均获接受。

本轮8站101..108按序WAIT_WORK/phase4、2秒测试驻留、模拟WORK_DONE、最终DONE/index19/count19，fault0/chassis_fault0。真实START确认到首次DONE约136.96秒（约2分17秒，含每站2秒驻留，不含真实扫码抓放）。最终电控估计(153,150)mm/yaw89.20°，未外部测量精度。运行阶段phase5数据连续解析正常，STOP发送0条，未发生超时；这轮验证了phase5兼容修复实机有效，不能把不同路线/不同电控固件的用时差全部归因于Pi优化，也不据此认定连续转弯功能已完成验收。

用户整个测试期间收到实时进度反馈；完成后服务正常退出且持久化本轮锁继续有效，未自动重跑。Mac忽略目录output/chassis_phase5_success_20261103/完整保留wire/prepared_live/result/status/summary和日志zip。实时journal监看SSH已退出；独立雷达继续运行。下一轮需摆回对应起点，再显式复位/准备，新键启动。源码未改，本轮未Git提交。

## 启动后假超时根因与phase5兼容修复（2026-11-03，最新，纠正前述推断）

用户反馈之前能跑、现在启动后停并连续超时。读取output/chassis_button/wire.jsonl原始RX并重新逐帧解码，确认最新19点任务route_id1793702522在18:42:07上传/START接受后，STATUS三个回复均约50ms返回、CRC正确且sequence/request匹配，但phase=5。Pi旧status_payload检查phase>4非法，_exchange吞掉ValueError继续重试，约3秒后误报超时并发STOP；电控已执行STOP并回ACK/state5/phase5，但同一旧解析器继续丢弃，错误报告“停止未确认”。实际收到的X从150→223→298→339，停止在第0点。此前将本次启动后故障描述为“STM32不回复”不准确，原始报文证实是Pi解析兼容问题；18:36待机真正无回复与本次假超时不能混为同一原因。

修复electrical_protocol.status_payload：phase按u8执行阶段原样保留，不把诊断阶段编号当route state范围限制；36字节/CRC/序号/请求类型/state0..6/reserved0/点索引检查继续保留，WAIT_WORK推进仍要求state3+phase4及真实作业条件。没有给phase5猜测控制含义，也未修改坐标或运动参数。electrical_link超时错误补充“收到但被解析拒绝的匹配ACK数量/原因”，避免再把协议拒绝混报为线路无回复。

真实抓包477状态/事件帧全部解析（470个phase0、7个phase5），新增实际RUNNING/STOPPED payload回归和非法state/reserved/index拒绝。Mac/Pi相关121项各自通过；3文件已部署，备份backups/phase5_20261103_5wirp7n9。修复后实机只发STATUS立即收到stateSTOPPED、phase5、route1793702522、point0/count19、估计(339,150)/90.31°、chassis_fault0，确认控制器已停；未上传、START、复位本轮或重跑。底盘服务inactive且本轮锁已消耗，车不在区1原始中心，下一次跑需重新摆放并显式准备。

证据Mac output/serial_diagnosis_20261103含wire/result/prepared_live/decoded/phase5_replay；Pi同名目录另有状态探测。独立原始探测旧解析器也复现同一ValueError，证明不是后台服务线程特有。系统另见持续欠压告警（包括18:42:07），应后续核查供电，但不能用该告警替代这次已证实的解析拒绝根因。最新电控C源码原下载路径已缺失，phase5完整业务语义待对方当前源码确认；兼容解码不表示已验证新版固件连续转弯能力。源码修改尚未Git提交。

## 待机STATUS超时恢复修复（2026-11-03，最新）

用户反馈按KEY1没有反应。确认旧PID4208在18:36:23 STATUS连续无回复后退出、无人监听，非雷达扫描延迟。修复共享wait_ready_key：只有尚未按键消耗本轮时，STATUS超时/MCU重启/传感器失效回到WAIT_CONTROLLER、重新建立新基线及Client；积压旧按键ACK后忽略，恢复READY后需要新按键。按键后的扫描/上传/运动失败仍不自动重试，持久化一次锁保持；同样用于正式入口。USB串口拔掉引发OS错误尚不在本次自动恢复范围。

Mac/Pi相关116测试各自通过，新增READY后掉线→恢复→忽略离线旧键→只接新键、启动时STM32无回复持续等待/可取消的回归。2文件已同步，备份backups/standby_reconnect_20261103_wpbgeu86。确认round_latch不存在，恢复底盘服务PID6377，检查为active/WAIT_CONTROLLER；CH340与CP2102均枚举、雷达运行，但STM32仍不回复STATUS，当前不能说READY。未发路线/START、未自动执行用户刚才的旧按键。已向用户询问主板上电与是否改过接线/烧录，等待实际确认。

## 正式流程也接入一键启动（2026-11-03，最新）

用户明确要求正式比赛流程同样按实体键自动启动。新增tools/competition_button_service.py、deploy/competition-button.service；复用chassis_button_service的预热/READY/新键过滤/一次锁，正式入口只允许显式真实后端，不回退底盘2秒模拟驻留。ElectricalClient新增run_selected_route：已选区后直接绑定上传/启动，不等待第二次按键；run_after_key复用该正式执行器。正式执行拒绝test_mode路线，仍以真实on_work反馈推进，等待未完成反馈/超时STOP行为保留。

正式本机配置需competition_backend=module:factory，工厂返回simulation=False及ready/plan/on_work。后端未配置或未就绪不会正式READY；plan须返回valid_for_motion=True且匹配route_id/start_zone的已标定路线，上传前完整几何检查，START前复核扫描。当前扫码/抓放/显示/统计实体后端未完成，所以正式入口部署但disabled，未伪装成可参赛流程；与底盘测试unit互斥，避免同UART双调度。现有mission_ros是外部后端框架，不擅自同时启动它抢占STM32底盘所有权。

Mac/Pi相关114项测试各自通过，新增两区正式已选按键直接启动、拒绝测试路线/模拟后端、启动前计划失效不START等测试；原正式无反馈不放行保留。7文件SHA256部署，备份backups/competition_button_20261103_0gd8vowp/before.tar。正式unit仅安装、不enable，不重启现场底盘。代码/说明尚未Git提交。

设备检查另发现：此前PID4208底盘待机服务18:36:23因STATUS连续超时退出，status.json=FAILED/No matching reply，尚无新按键运动记录。当前不能说仍READY；未因这个错误自动重启、发车或复位本轮。电控供电/连接现状未确认，需重新取得STATUS后再显式重启待机。本轮新增正式入口未发送任何运动命令。

## 实体按键一键启动封装（2026-11-03，最新）

用户要求提前启动雷达和按键监听，只按实体键就自动跑。新增tools/chassis_button_service.py和deploy/chassis-button.service、CHASSIS_BUTTON_START.md。服务独占电控UART、复用现有雷达HTTP；未运行时按本机显式配置启动seconds0/6Hz采集。USB设备/STM32未上电时等待，确认新鲜雷达+STATUS后READY。READY前旧按键请求ACK后忽略；READY后KEY1区1/KEY0区2确认一次，即持久化本轮锁→选区后新扫描窗口（仍4秒）→新八段规划→几何校验→请求绑定上传→START→8站闭环→回家。没有电脑二次放行，无旧路线预缓存启动。保留100rpm巡航上限/20rpm短连接、两区中心及+90°约定。READY后请求失效/MCU重启不自动重试。

本轮锁output/chassis_button/round_latch.json在完成、失败、进程重启后继续生效，不自动二次发车。下一轮测试显式停止服务、--reset-round、重新启动（命令见CHASSIS_BUTTON_START.md）；复位前日志/结果会归档history，route_id高水位保留。UART原始日志10MB×4轮转，所有状态转换持久化status.json。正式比赛流程仍未补全：此服务强制--test-only-route，各站2秒模拟作业；扫码/抓放/任务显示和计数不能冒充已完成。正式run_after_key仍要求真实作业反馈。

原始全国命题tasks_national_2027.pdf第7页核对：裁判统一发令后，明确标识的一键式启动、一次启动机会；第1页要求完全自主运行。封装遵循该启动结构，4秒是工程采样策略不是比赛规则；本场启动窗口仍按正式通知/裁判。未据此宣称整车比赛合规验收已完成。

5文件SHA256部署备份backups/button_service_20261103_mhlpj2v_/before.tar；实际私有配置config/chassis_button.local.json保持Git忽略，Mac副本output/button_service_deploy_20261103。Mac/Pi相关107测试各自通过（包含旧键过滤、无键不启动、过期扫描拒绝、MCU重启拒绝、持久化一次锁）。user unit已enable，loginctl enable-linger cuiguo成功（Linger=yes，直接本人操作无需sudo）；主进程PID4208，已实测READY、STATUS通信正常，检查待机仅0x14/没有路线上传或START；现复用独立雷达PID1310。没有断电重启实测，不将enable/linger等同完整冷启动验收。本轮未发车，按新实体键可直接运行；此时服务已处于有运动能力的待机状态。新增封装尚未Git提交/推送，前次fd430ae仍在origin/main。

## 优化路线区1实车全程完成（2026-11-03，最新）

用户要求启动并按KEY1直接跑完全程，按已确认区1中央(150,150)mm/+90°执行。Pi刚重启，电控CH340→ttyUSB0、雷达CP2102→ttyUSB1均重新枚举、无人占用；优化源码SHA256与Mac一致。雷达以6Hz/seconds0恢复，PID1310，runtime_zone1_20261103_rvk42nrz，静止校准未改，恢复Mac8766隧道。收到真实KEY1 request1/tick3377，AREA_ACK OK后重新扫描规划，16航点、8作业站，100rpm巡航上限、20rpm首尾短连接、turn15，使用controller_deceleration新默认。

上传route_id1793701103（0x16共354字节）及START获确认。八站101..108依次WAIT_WORK、2秒测试驻留、模拟WORK_DONE，最终DONE/index16/count16、fault0/chassis_fault0。START首次RUNNING到首次DONE实际约173.22秒（约2分53秒，含16秒测试驻留；不含夹爪及视觉），最终电控估计(153,153)mm/yaw88.89°；无外部绝对定位精度测量。路线不同于上一轮区2，不能把275→本轮时长全部归因于压点。实车完成基本跑图目标，不代表含机构作业的180秒目标已达到。

日志已回收Mac/Pi output/chassis_zone1_optimized_20261103/：全部TX/RX_RAW、prepared_live/result/profile/summary、日志zip；脚本已退出释放UART，未自动重跑。独立雷达继续静止预览，无移动定位。此轮新增实际运行记录尚未Git提交。GitHub前次代码提交fd430ae已在origin/main；其他视觉未提交改动保留。

## 减少直线多余停车（2026-11-02，最新）

用户实车反馈32点中存在不必要停顿、整车目标180秒且还需夹爪作业。核对当前C源码：所有航点均等CHASSIS_DONE停稳，work_id=0不等于不停通过；原Pi额外120mm接近点及直线网格中间点引入停起。compile_chassis_test默认stop_strategy=controller_deceleration，取消额外接近点，按同向共线/同yaw/同速度压缩中间点；保留拐点、折返、8站、首尾短距低速连接和完整几何检查。V1没有独立接近速度字段；默认approach_rpm仅限制首尾短连接、approach_distance_mm只供显式split_approach历史对比。没有改下载目录固件或烧录，没有清掉闭环停稳/位置容差。

上一轮区2真实保存快照32→20点（少8个接近点和4个直线节点），相同完整路径。仍有12个非作业转折/限速连接点会停，不宣称仅8次停车。保持巡航100rpm，末段20rpm短连接。新增tools/chassis_route_benchmark.py用保存计划编译对方真实C控制器，在模拟UART/CAN/轮子/IMU中做同路线A/B：旧32点238.531秒，新20点100rpm179.921秒，120rpm177.977秒，均8站DONE并验证四轮反馈≤约1rpm/停稳≥250ms、观察到减速。模拟不是实车预测；原实车274.91秒，不能声称改完达到整车180秒。提上限100→120仅模拟省1.944秒，主要问题仍是停起及distance_kp=0.20的末段渐近降速，后续电控需轨迹前瞻及实车制动参数验证。

Mac和Pi相关93项Python各自通过；两区KEY模拟均8站完成；堵转注入新旧路线均故障并确认STOP。10个代码/测试/配置示例/文档文件已同步Pi并逐一SHA256核验；备份backups/route_optimization_20261102_n8c1rfdq/before.tar，收尾context另同步。完整对比、profile_zone2.json和优化路线参考在output/route_optimization_20261102（忽略），旧实机日志不覆盖。面向电控说明ROUTE_TIMING_REVIEW.md。没有启动新实车运行。检查SSH可达cuiguo/aarch64且工程存在，但无/dev/serial/by-id且无雷达/电控运行进程，当前USB设备未枚举；不能假定旧雷达后台仍在线。

## 换场地后区2全八段100/20rpm完成（2026-11-02，最新）

用户换地方后要求直接运行现有代码、等待KEY0并完整跑8段，确认仍在区2中央、车头+90；要求巡航提高及接近提速，采用巡航100/接近20rpm（turn15）。未重读无关资料，复用上一轮live_zone2程序，只改速度和独立日志目录，去除临时离线模拟等待门。独占电控UART前核对无其他运行程序，不改固件/视觉。沿用程序现有开局扫描/八段规划和几何验证；本次计划因当前阻断证据不同生成32点，较上一轮28点有绕行，不能将不同路线总耗时直接作为速度对比。

目录Pi/Mac output/chassis_zone2_100_212723/（忽略）；profile含实测完整外廓和起点(150,2250,90)，脚本live_zone2.py。接收真实KEY0 request2/tick804034、AREA_ACK OK，0x16路线route_id1793626055获得READY，0x11 START获RUNNING。各站101..108依次WAIT_WORK/2秒停留/模拟WORK_DONE，最终DONE/index32/count32、完成8站、无fault/chassis_fault。实际START到DONE约274.91秒（4分35秒）。最终MCU估计(153,2247)mm/yaw89.82，相对目标估计差4.24mm/−0.18°，未外部测量终点精度；用户本轮现场反馈尚未收到。

本轮全部TX含STATUS及RX_RAW均在wire.jsonl，结果/准备快照回收Mac，脚本退出释放UART，未自动重跑。车最终仍在区2，雷达运行独立静止预览；移动定位仍未接入。不将本轮100rpm上限解读为全程实测轮转速恒为100。

## 区2全八段90/15rpm完成与通讯日志交付（2026-11-02，最新）

用户评价上一轮区1“很完美”，要求拿完整发送日志发给电控；助手仅生成本地文件供转发，未代发消息。区1日志包output/electrical_logs_for_stm32_20261102/区1_60rpm_八段完整日志.zip，含wire/result/prepared/saved_route、可读报文说明、实际TX筛选和SHA256。原记录器仅保留非STATUS TX（11条）和解码ACK/EVENT，STATUS的原始TX未逐条记；根据918条STATUS ACK的类型/sequence重建查询另存并明确标注，不冒充原始抓包，丢ACK的查询无法恢复。此边界已告知用户。

用户已将车放到区2中央，明确车头仍向网页上方+90°；本轮起点(150,2250,90)。请求巡航90rpm（接近仍15rpm、turn15）、KEY0触发完整8段。新建忽略目录output/chassis_zone2_90_20261102/，实际profile与脚本live_zone2.py已部署，增加全部TX（含STATUS）及RX_RAW记录。切换雷达服务到已有区2静止预览配置，重新扫描生成区2批次，未沿用区1起点。初次收到request2后准备期间MCU重启，request倒退至1；Client正确停止且未上传/START，原始记录保留attempt1_mcu_reboot，随后新Client接收本次真实KEY0 request1/tick1481，ACK OK。不得把此次重启当作运动故障或已执行一次。

最终本轮plan_id5b2b533c396f4c4ca8f0c7aee03cb71c，28点，真实C模拟90/15完成8站（209.259秒模拟时钟）；新扫描几何核验通过。0x16上传route_id1793625203、28点、READY后0x11 START接受；各站WORK_REQUIRED/WAIT_WORK→2秒停留→模拟WORK_DONE均按1..8完成。最终DONE/index28/count28、chassis_fault0/fault0。实车START到DONE约236.35秒（3分56秒），最终电控估计(153,2251)mm/yaw88.88，相对目标估计偏差3.16mm/−1.12°；未外部测量绝对位姿。用户尚未提供本轮现场终点评价，不据估计差声称定位精度。

新轮result/prepared_live/wire均回收Mac同名忽略目录；脚本已结束释放UART。当前车位于区2回程终点，雷达服务仍是静止估计预览，未接移动定位。区2+90°与实车外廓已确认；此前模板留null为旧状态，不要再反复询问车头约定，只有摆放改变时才核对。

## 完整八段60/15rpm实车一次完成（2026-11-02，最新）

用户要求提高速度并跑完整8段，确认车已回区1中央、车头+90。先以巡航30/接近10rpm执行完整路线，真实KEY1 request2、route_id1793624290、28点；前2站完成，第三段point_index7时电控FAULT：route_fault1、chassis_fault4、address2。Pi立即STOP且ACK OK，保留FAULT，未自动重启。对方源码fault4=CHASSIS_MOTOR_ERROR，在0x3A状态反馈(flags & 0x0D)!=0x01触发；UART未回传原始flags，不能据此断言具体堵转/供电/失能原因。30rpm轮证据output/chassis_full_20261102/，Pi/Mac均保存，未完成8段。

用户随后说明小问题已处理，要求幅度更大提速并只复用刚才算好的8段、不要重新扫描；明确答复已放回区1中央、报警清除。执行前STATUS确认MCU重启后EMPTY、无fault；收到新的真实KEY1 request1/tick4351，AREA_ACK OK。使用上一轮prepared_live.json的原路线，逐点X/Y/yaw/work_id、顺序、起点和段映射完全一致；只将30rpm改60、10rpm改15（turn仍15）。保存配置output/chassis_saved60_20261102/saved_route.json；此轮脚本不调用acquire_plan或HTTP雷达接口，按用户授权复用原静止障碍快照，不宣称实时动态避障。

真实下发0x16共594字节、28航点、route_id1793624629绑定新KEY1请求，READY后0x11 START接受。8个WORK_REQUIRED/WAIT_WORK均按顺序闭环到点、轮询停留2秒并模拟WORK_DONE。最终DONE/index28/count28，test_stations_completed8、fault0/chassis_fault0。START到DONE约246.50秒（4分6.5秒）。最终MCU估计(153,153)mm/yaw88.75，相对目标(150,150,90)的估计差4.24mm/−1.25°，未做外部位置测量；本轮用户现场完成情况还待反馈，不宣称定位精度或长期稳定。

实车脚本Pi/Mac忽略目录output/chassis_saved60_20261102/live_saved.py，模块运行，已结束释放UART；该脚本无持续监控/自动重跑。启动前真实C模拟同28点、60/15速度完成8站，约216.37秒为模拟时钟，不能混作实车耗时。result/prepared_live/wire.jsonl均回收Mac；源码未改固件。下一轮需新的实体KEY1与单调递增route_id，并先核对实际起点/朝向和最新场地条件；不能因本轮成功静默复用快照到新条件，也不把旧18cm圆形模型误报重新引入。雷达仍为原独立静止服务，没有移动定位接入。

## 首段第二次实车重跑完成（2026-11-02，最新）

用户现场确认第一次运动方向正确，并要求重跑；明确回复已将车放回区1中央、朝二维码区。第二轮STATUS显示MCU已重启（tick约57977、EMPTY），重新收到真实KEY1 request1/tick5617并ACK。该轮新的扫描显示后续原料等段no_path，但首段8→5仍path；原程序要求全8段ready导致没有上传。已修正单段测试：acquire_plan(first_stage_only=True)、compile_chassis_test(stage_limit=1)只要求首段可达，其余段不参与编译/几何校验；完整8段模式仍要求全部可达。增加“后续无路时仅首段允许、首段无路仍拒绝”的测试。Mac/Pi相关84项通过。

随后在同一未重启MCU上重ACK本轮真实KEY1，重新扫描；首段几何检查通过，上传与第一次相同4点（114字节），本轮MCU boot内route_id1。START接受RUNNING，逐点完成；WAIT_WORK停留2秒后WORK_DONE、DONE/index4/count4，最终估计(346,1197)mm、90.08°，无故障，约45.3秒。只跑了第一段，未自动重试/重跑运动。第二次实车方向/绝对位置还未收到用户现场评价。

两轮文件分别保存Pi output/chassis_first_segment_20261102/run1_completed与run2_completed，result/prepared_live/wire.jsonl已回收Mac对应目录（忽略）。最新脚本已退出释放UART。脚本恢复历史选区时现在要求初始route_id0；已完成任务不能再用消耗过的选区上传新任务，需新的实体KEY1。MCU重启后不继承旧tick，恢复判断仅取最新实际START_AREA之后的日志。当前车辆再次位于首段末端，旧区1静止雷达叠加已不适用于当前位置；下一次跑起点路线前仍须重新放回并确认，不能自动重置位姿后发车。

## 第一段实车闭环已完成（2026-11-02，最新）

用户已反复授权首段实车执行，随后完整确认底盘长286/宽257mm，右侧雷达板位于前后中部、总宽294mm，即单侧超出37mm。配置参考点为原底盘中心（并非加板后的包络中心）：front/rear=143、left=128.5、right=165.5mm，margin=2mm；起点按用户确认的区1中央(150,150)mm/yaw90。完整配置Pi与Mac output/chassis_first_segment_20261102/profile.json（忽略）。不将165.5mm当雷达旋转中心外参。

修正矩形模型后首段几何检查通过。固定航向外廓检查覆盖完整平移扫掠的静态禁区和占据格；同时保留原0.18m中心线静态障碍余量，不再将矩形外接圆替代真实外廓而误报贴边车道。check_boundary=false仅在矩形边界已通过时调用，旧无外廓路线保持圆形边界逻辑。first_segment_only只上传首段4点，run_closed_loop支持1站或8站，DONE判断按实际段数。

真实UART已发送绑定原KEY1请求1的0x16，114字节、route_id1，收到READY；随后0x11 START接受RUNNING。目标(350,150)6rpm→(350,350)12rpm→(350,1080)12rpm→(350,1200)6rpm，所有yaw90，末点work101。实际ACK/EVENT反馈逐点推进，最后WAIT_WORK后2秒模拟作业释放，DONE/index4/count4，无chassis_fault/route_fault。最终STM32估计(345,1197)mm/yaw91.37，相对末目标偏差约5.83mm与1.37°，是电控估计，未获得用户现场方向/位置反馈，不宣称实测精度。只执行首段，没有后七段、相机或机构动作。

实车脚本为忽略目录output/chassis_first_segment_20261102/live_first.py，以模块方式运行（python3 -u -m output.chassis_first_segment_20261102.live_first），直接脚本路径导入会失败且不会发指令。已结束并释放UART。脚本从本次实际KEY1日志恢复选区，经当前tick未倒退、AREA_ACK OK再次核验；不在未知重启/位置变化后重跑。下轮route_id须递增，车辆已在首段末端，不能再次把起点设区1中央。

99项相关Mac/Pi测试通过，真实C模拟首段1站DONE（约44.96秒模拟时钟），7文件SHA256部署备份backups/chassis_first_20261102_ke8lrz/before.tar。实机prepared_live/result/wire.jsonl在Pi output/chassis_first_segment_20261102/且已回收Mac。本次新扫描占据格7，完成首段用约46.8秒（START到DONE）。当前雷达服务仍使用旧区1静止预览位姿，车移动后该叠加不再是真实定位，继续规划前须重新确认位姿/扫描；不把当前红格当移动后实测障碍，不把离散MCU状态直接接作连续定位。

## 实测车体外廓与边界模型修正（2026-11-02，最新）

用户确认底盘前后长286mm、左右宽257mm，并提供实车照片，明确雷达在车身右侧前后居中位置，安装板右向突出未计入257；后续已确认总宽294mm，右侧超出37mm，详见最新实车记录。右侧是车体系−Y，区1车头+90°时对应照片同向地图+X。照片无可靠尺寸标尺，不由像素猜伸出量；雷达旋转中心偏移、板的最远外缘和传感器零度安装角是不同参数，后两种位置不能互相顶替。单看底盘，区1中央摆放的两组边距21.5/7mm，不代表完整外廓已确认。不要凭这个不完整尺寸自动START。

新增pi_vision/chassis_geometry.py，支持相对于车体定位参考点的前/后/左/右四个外廓距离及margin_mm，包含非对称安装板。测试profile可加footprint：以固定航向矩形验证场地边界，早期使用外接圆校验障碍，后续已改为保留原中心线余量并验证真实矩形扫掠，occupied检查保留（详见最新记录）。未传footprint时沿用旧逻辑；拒绝不完整外廓及航向变化，避免只查端点漏掉转向扫掠。static_blocked_edges仅增加默认true的check_boundary选项，雷达运行配置不改。

新增13项外廓/板方向/旧模型误报/真越界/障碍保留测试，Mac全套374 passed、172 subtests。7文件已部署Pi并SHA256核验一致，备份backups/chassis_geometry_20261102_XrkeOr/before.tar；Pi全套373 passed、1 skipped（缺Node.js，Mac已过）。外廓参数尚未写入实车运行配置，待板侧向尺寸，不能将测试夹具中的无板矩形直接用于本车。未发送START。此次SSH快照仅见雷达占用串口，旧持续监听进程已退出，不要声称仍在实时监听。

## 实机KEY1已接通、首段运动被几何检查阻止（2026-11-02 20:10后）

STM32上电后，20:06:58首次STATUS得到匹配ACK OK、EMPTY/无故障上报。20:07:00.854收到START_AREA：request_id=1、tick=4522、area=1、key=1；Pi发AREA_ACK，20:07:00.955收到匹配ACK OK。持续STATUS正常，先前0字节现象已随STM32上电消失。监听仅发送0x14和0x15，未上传或START。

用户要求持续实时联调，并已授权先执行第一段；随后确认车在区1正中央、不要再追问。因此起点按名义区中心照片坐标(150,150)mm、yaw=90记录。现有0.18m车体余量检查拒绝该起点及起步连接（中心离边界0.15m）。未获得实际车体尺寸，不能删除/缩小保护绕过检查。仍未下发运动。新扫描八段规划已成功、占据格7、首段8→5；source_seq3235，plan_id16a57ed452894cccb23d07f31fe75790，文件output/electrical_live_20261102/fresh_plan.json，需运动前重新采集，不复用陈旧证据。

当前实时监听为本会话exec session43667，独占电控USB串口；本地脚本output/electrical_live_20261102/monitor.py，通过SSH python stdin执行，Pi日志output/electrical_live_20261102/session_*.jsonl。每10秒STATUS，收到START_AREA仅AREA_ACK。后续执行程序前必须先有序停监听并核实串口释放，不能第二个进程抢读；旧按键已ACK，不假设新Client还能收到它。保持不启动底盘，直到起步几何能基于实际证据验证。

## 接线后设备检查与雷达恢复（2026-11-02，最新）

用户接入电控后检查发现Pi刚重启（uptime约1分钟），旧雷达服务/隧道已退出。新增电控USB串口为 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`→ttyUSB0；雷达固定CP2102路径现→ttyUSB1。必须用by-id，不能继续将ttyUSB0当雷达；checked_port按真实固定路径保护，已可正确打开电控适配器。

对电控执行115200/8N1无动作STATUS：CLI重试超时；另记录5秒原始收发，两条0x14查询发出，RX为0字节、没有合法回复。未上传路线/START/WORK_DONE，也未监听确认实体按键。结论仅USB枚举及端口可打开，STM32链路未通；待核对STM32上电、UART4 PC10/PC11交叉接线与共地、烧录版本/波特率。

雷达恢复为PID1299，独立运行目录 `/home/cuiguo/ydlidar_work/live_view_20261101/runtime_reconnect_20261102_miaq5p9z`，6Hz、--seconds0，沿用两区原校准，当前区1；新建Mac8766 SSH隧道。实际HTTP检查seq118→121→125，valid=true，约577–583点，数据持续更新。仍非开机自启。日志/command.json/pid在该运行目录；本地证据output/electrical_reconnect_20261102/（忽略）。旧PID/ttyUSB编号仅历史快照。

## 电控统一照片同向坐标（2026-11-02，最新）

用户已确认采用照片同向网页的场地坐标：左下原点、右+X、上+Y、逆时针为正，两区共用。区1车头沿通道朝二维码区为+90°，双区示例已填写yaw/heading=90；区1/区2实际车体X/Y及区2朝向未确认，仍留null。不要再将旧地图−90°当作电控配置。

电控坐标名field_photo_ccw；原雷达/规划继续使用field_nominal以保持静止校准。新旧转换X=2400−X、Y=2400−Y、yaw加180°归一化。配置本身已是电控坐标，编译器只旋转原路网节点并米转毫米，不二次旋转配置中的+90°。通用/测试编译器要求显式coordinate_frame，编码器拒绝旧/缺frame路线。底盘几何检查反变换到原图；STATUS与到点判断使用新坐标，pose_snapshot标注field_photo_ccw。UI旋转开关不参与协议，雷达pose仍不冒充车体pose。

增加electrical_coordinates.direction_review解码实际下发payload，通用Client保留last_upload_review；底盘测试START前打印并保存每点先转/前后/左右动作。Mac全套361 passed、172 subtests；相关68项通过。electrical_direction_check用原电控C与模拟轮子检查18种方向/顺序均通过，imu_yaw_sign保持−1。新坐标区1+90°和区2模拟−90°均完成按键→8站闭环回程，25航点、24子动作并观察到减速；区2模拟角不是实际摆放确认。证据output/electrical_direction_20261102/（忽略），均非实车验收。

本批18文件已同步Pi /home/cuiguo/competition_vision并逐一SHA256核验，旧文件备份backups/electrical_photo_20261102_RJnDAI/before.tar。Pi全套360 passed、1 skipped（缺Node.js；Mac已过），Mac361 passed、172 subtests。文档收尾另行同步。运行中的独立雷达未重启、未改校准，在线valid=true、区1。检查时雷达PID80798仍运行，by-id仅有雷达CP2102，电控USB-TTL尚未出现。下一步核对新增串口→两键无动作收发→填实际车体起点/区2朝向→短距验证方向再跑全程；未发真实运动命令。

## 雷达后台不限时运行（2026-11-02，最新）

用户明确要求恢复雷达并取消固定运行时限。确认旧PID42082及SDK已退出、串口空闲，旧summary记录子进程退出0；原--seconds3600到期。`tools/lidar_view.py`默认seconds改0表示不限时，显式正数仍可用于有界诊断，拒绝负数/非有限值；原扫描失效判定、有限日志/前300圈保存及SIGTERM有序退出不变。

已同步Pi正式工程与独立运行目录；本次后台PID80798，以独立session启动、6Hz、--seconds0，运行目录 `/home/cuiguo/ydlidar_work/live_view_20261101/runtime_continuous_20261102_Y74oLP`。恢复旧summary退出前的启停区1/对应校准；两区start_profiles保留，stationary_preview.json顶层同步区1。旧源码/配置及工程副本在运行目录before/，未操作相机/电控/底盘。Mac8766隧道复用，网页 http://127.0.0.1:8766/。35项导航/服务/批次测试通过；实际在线检查见本轮忽略目录output/lidar_continuous_20261102。此为取消定时退出，不宣称已增加断电自启或故障自动恢复。

## 最新交接：新版按键协议与纯底盘测试（2026-11-02）

用户提供新版 `/Users/jzxzhou/Downloads/car_code1 2/`，要求测试和通用正式接口都接KEY0/KEY1选区并部署Pi，准备联调。用户选择 **USB转TTL**，不用修改GPIO ttyAMA0控制台。先读 `ELECTRICAL_COMMUNICATION.md`、`CHASSIS_ROUTE_TEST.md`。旧拟议nonce/boot_id/revision协议已完全撤下，不兼容新版命令含义。

新版实际协议：KEY1(PA15)→启停区1、KEY0(PC5)→启停区2；0x82 START_AREA是12字节request_id/tick/area/key/reserved，Pi以0x15 AREA_ACK确认，0x16 UPLOAD_FOR_AREA携带request_id＋路线内容。最大payload1304/整帧1314，仍64点。按键重发去重、扫描中换区丢旧结果、绑定上传；KEY0在活动态停车，KEY1在活动态忽略。按住任一键禁止START，Pi最多等10秒松键/IMU就绪。主控重启后需重建Pi Client重新按键；无boot_id，不声称检测所有快速掉电重启。

通用 `ElectricalClient.wait_start_area/plan_after_key/run_after_key` 已实现：先ACK后调用规划，正式on_work回调只有真实True/False才发送完成，None继续等；无回调不自动成功。底盘测试 `tools/electrical_route_test.py --wait-key` 复用相同Client，按区选择本机两组实际起点配置，接收新扫描→8段规划→绑定上传→START→8站闭环到点、2秒模拟工作停留→回实际起点。真实执行需--execute --test-only-route；不涉及视觉/夹爪，原固件Route_OnWork为空。完整mission_ros调度未同时接管底盘。

测试固定车头为地图轴向，增加首尾实际起点连接与末段低速接近点，检查占据格/静态禁区；不把雷达pose当车体起点。对方已有编码器/IMU闭环、剩余距离降速、四轮约±1rpm持续250ms停稳后DONE；每个航点都会停车。当前V1位姿为子动作完成后的离散累计，不是连续定位。测试仅开局静止障碍快照，无运动中动态避障/重规划；UART断连固件不自动停车，Pi STOP可能无法送达，保留KEY0本地取消。

Mac全套 `346 passed, 172 subtests passed`；通讯/按键/测试相关53项。新版真实route.c/route_uart.c/route_keys.c的20项C主机测试通过。实际route/heading/motion/IMU源码加模拟UART/CAN/电机，两个启停区按键请求链都完成25航点、24子动作、8次停站回程，并观察到降速/停稳。旧版故障模拟及Python掉线/超时/取消仍通过。报告 `output/chassis_route_test/`（忽略）；这些不是真实UART或整车验收。没有修改/烧录下载目录里的固件。

已部署Pi `/home/cuiguo/competition_vision`：26个本轮通讯/底盘测试模块、测试、文档、示例配置和已有规划依赖逐文件SHA256核对一致；旧同名文件备份 `backups/electrical_keys_20261102_28vEA6/before.tar`，含before_manifest.json和部署manifest.json。未覆盖Pi视觉工作区/实际配置，未重启独立雷达服务，未打开真实UART或发运动命令。Pi全套 `345 passed, 1 skipped`（Node.js网页测试Mac已过），两个CLI帮助入口通过。Mac部署记录 `output/electrical_deploy_20261102/` 忽略。两区实际车体起点模板留空，不伪造标定；可先直接运行electrical_comm watch-areas联调两键。模拟工具也已部署，但对方C源码及clang只在Mac用于交叉模拟，未在Pi重复编译固件。

设备只读快照：既有SSH密钥/主机校验、临时172.16.24.76可用；唯一by-id为雷达CP2102→ttyUSB0，电控USB-TTL尚未发现。用户后续插入后重新核对稳定路径，TX→PC11、RX←PC10、GND共地、3.3V逻辑；不猜ttyUSB1。相机/底盘未操作。下一步是USB-TTL枚举→STATUS/两键无动作联通→填两组实际车体起点与朝向→纯底盘测试；不重新要求视觉/夹爪验收。

## 目标与当前阶段

智能+工程创新赛道机器人视觉。视觉主线正在把可用工程迁移到 Ubuntu 24.04 + ROS 2 Jazzy；2026-10-23 用户另行明确提出雷达开发并将雷达接入树莓派，当前雷达工作从设备识别与扫描基线开始。Mac 开发、传代码，树莓派运行和测试；树莓派无屏幕，通过 SSH 操作。

当前重点：真实视觉单相机后端、中央单目标选择及圆顶独立中心已部署Pi；top_face物料模式已实测；最新圆环有界测试及拍照均已结束，18:45检查后相机释放，无常驻采集。任务/ROS及局部对准软件已有，真实电控与机械标定未接入。最新证据与命令见文末“顶面定位已部署并实测”，历史PID/状态均为快照，不代表六色或整车验收通过。

用户最新要求：助手直接搭任务框架，不再安排反复拿放物料、扫码或网页显示。保留现场调参、坐标标定、真实动作反馈接口；定位精调等真实安装后再做，不为桌面 3 px 目标改阈值或冻结坐标。朋友截图仅作思路参考，不要求获取对方完整代码才能继续本项目。

推进计划见 `DEVELOPMENT_PLAN.md`，视觉顺序为任务调度核心/模拟 → ROS 2 收发 → 真实视觉结果与电控后端 → 固定安装标定 → 场地与整车验收。用户报告电机/电控进度为零，不假定底盘或机械臂能执行；先完成独立可运行的软件主线。2026-11-01 用户另行明确要求雷达实时观察和场地避障参考实现，已启动独立雷达观察工具，见文末。仍保留漏检、额外颜色和 USB 掉线等未解决事实，不将模拟通过当作实车通过。

相机安装位置尚未固定、物料摆放在变化。此前静止抽样只作诊断，未隔离相机移动/视角/摆放影响，不能作算法定位稳定性验收，也不能确认全部问题均由摆放导致。暂停重复摆放和像素精调；时效/有效性语义保留，固定安装后再做精度与机械标定；这些未验项不阻塞当前任务框架。

采集时间/有效性/超时/故障的软件完善已实现、同步并部署到 Pi，见下文“时效与故障处理”。Mac/Pi 视觉软件测试均为 130 项通过，软件故障注入和真实相机在线检查分别留证；这不替代扫码、实体移出/恢复、固定安装精度或整车验收。

## 当前任务框架

- 新增 `pi_vision/mission.py`：严格任务码确认、按容量 1/2/3 编排两批任务、逐动作反馈推进、托盘占用和计数、第二批同色码垛位置映射、取消/故障停止、过期/重复/迟到结果拒绝。既有 `state_machine.py` 是视觉模式切换器，保留不改；新调度层不直接接旧串口，不开启硬件。
- 新增 `pi_vision/mission_demo.py`：无需相机/电控完整模拟，示例 `python3 -m pi_vision.mission_demo --capacity 3`；输出明确标记 `software_simulation_only`。示例完整计划为 59 个步骤（扫码另计），容量 1 为 75 步；模拟抓取装车 12、放置 12、码垛 3，不是官方得分或真实成功次数。
- 新增 `pi_vision/mission_ros.py`：schema 1 JSON 的 ROS 2 任务调度节点，指令/反馈/状态/控制接口；模拟主题 `mission_sim/*` 与非模拟 `mission/*` 隔离，默认不自动启动。不是已接相机的 ROS 视觉节点，也不是电控驱动；此次没有推翻或接管下文已有 STM32 主导方案。任务计划可作主控实现的可执行参考，最终只选一个流程所有者。
- 接口与运行说明：`MISSION_FRAMEWORK.md`；新增 `tools/mission_ros_smoke.py` 可用真实 ROS 2 pub/sub 模拟后端跑通流程，不访问相机/串口/电机。位置都是符号目标，无编造地图、毫米距离或标定值。真实后端必须拒绝未标定/过期目标，并独立负责本地停机保护。
- 新增 `tests/test_mission_framework.py`，覆盖全部三色组合与容量、两批/码垛语义、确认和去重、目标有效性、反馈和计数、超时/取消/停止确认、相机会话变化、模拟与真实隔离。Mac 视觉范围全套 `152 passed, 172 subtests passed`；Pi `151 passed, 1 skipped`（缺少 Node.js 的网页 JS 模拟，在 Mac 已执行）。硬件未接入项明确后置，不继续安排人工逐项验收作为门槛。
- 已在 Pi 的 ROS 2 Jazzy 通过真正的指令/反馈/状态主题进行纯模拟：正常例收到 60 条命令（扫码＋59 步），最终 completed，模拟计数抓取 12/放置 12/码垛 3、托盘空；抓取失败例收到 6 条命令，最终 fault，停止反馈确认，未计任何抓放成功。证据 `output/material_diagnostics/mission_framework_20261031_Al67eI/normal.json`、`failure.json` 和对应日志。测试进程用独立 ROS domain 73、仅本机发现，未启相机/网页/串口/电机，结束后退出，不留后台任务节点；不代表真实抓放或整车通过。

## 真实视觉 ROS 2 后端（2026-10-31 已部署）

- 新增 `vision_backend.py`（请求/观测适配）、`vision_ros.py`（ROS 2 节点）、`tools/vision_ros_smoke.py`（有界检查）、`tests/test_vision_backend.py`。复用既有 FrameStreamer 和 HSV；stream 仅新增默认开启的 `preview_enabled`，此后端关闭预览绘制及编码，无网页/串口/动作端。物理相机一个采集入口，依次承担扫码与物料角色；双摄尚未接入。
- `mission/command` → `mission/feedback`，只接真实模式 schema 1 请求；支持 `read_task` 新帧候选和原料区 `scan_material` 的指定颜色。保留原图坐标、框中心点定义、来源帧/会话、主机读帧完成时间和 HSV 哈希。空目标、多目标歧义、请求前旧帧、重复/过期指令、处理/相机故障均不伪装成功。粗加工重抓和 `locate_position` 仍要求位置身份；新增可选 `NumberedRingDetector` 环内数字模板，必须同时显式提供 `--position-digits --position-orientation`，否则仍失败关闭。
- PDF 平面图（T 第 5 页，A 第 20 页）给出 `580×150 mm` 长条、三目标沿长边排列且两端间距 `300 mm`；每个目标含 1—5 号同心评分环和中心位置数字，图上数字并非统一朝向。ICAROB2024 论文的相关方法是 HSV/二值形态处理后筛轮廓并拟合最小外接圆。本实现沿用现有多同心轮候选检测，在数字裁剪中先去除长横/纵尺寸线，再逐点比较四向模板；位置数字与评分环号分开，逐点输出位置号、环中心候选、环半径和数字方向。ROS 仍要求至少 3 个新鲜帧且跨度达到 0.2 秒。Mac/Pi 合成 PDF 布局场景能识别 3/2/1（含 180/180/0 度方向），缺字/重复仍无效；该几何测试不代表真实打印字体、透视、地面反光或实物验收。
- 物料追踪此前未实现：`MaterialStability` 仅用于短暂遮挡时维持串口上报中心，不提供跨帧身份。新增 `MaterialTracker` 按颜色、帧间位移和时间间隔做保守关联，并在 `/state` 与 ROS 目标中输出服务会话内的 `track_id`/`tracking_status`。短时遮挡期间 ID 可保留，但空帧目标列表仍为空且旧坐标不复用；超时、颜色变化或关联歧义会结束旧 ID。位置和物料坐标目前均为左上原点的原图像素，`metric_coordinates_valid=false`、`calibration_required=true`；没有输出/伪造毫米坐标或机械抓取点。
- 停止或后续动作命令关闭识别，但不发送整车 `stopped=true`；整车停止仍由动作端负责。此视觉后端不启动任务主控，不决定 STM32/树莓派哪个拥有整车流程；局部对准软件见下一节，真实坐标标定和机械动作仍未接入。节点启动后持续采集、算法按请求启停，退出释放相机；不能与网页服务或 qr_check 同开。
- Mac 全部视觉范围测试 `177 passed, 172 subtests passed`；Pi `176 passed, 1 skipped`，跳过缺少 Node.js 的网页 JS 模拟（已在 Mac 执行）。生成图像经真实检测器/采集线程/ROS 收发：物料 10 条观测、二维码 13 条，均满足工具检查；不属于真实场景识别验收。
- Pi 真实相机 ROS 观测：物料约 5 秒 17 条观测，包含 ok/no_target，采集 49 帧、错误 0；未核对当前物料真值，不能给出正确率。扫码约 5 秒 8 条 no_target，采集错误 0、无任务码，未安排用户展示码，因此仅确认空观测通路，不能算本轮合法码识别成功。两个检查均无 JPEG、停止后算法全关，进程退出后 fuser 确认相机释放；无后台节点遗留。
- 上述检查使用独立 ROS domain 74、LOCALHOST 发现，没有运行任务调度或动作后端；生成/真实证据分别在 Pi `output/material_diagnostics/vision_ros_20261031_N283rO/` 的 `generated_material.json`、`generated_qr.json`、`live_material.json`、`live_qr.json` 及日志。该目录被 Git 忽略。部署前核对 Pi 原 stream 与 Mac 改前 SHA256 完全相同，备份 `backups/vision_ros_20261031_qB4ucB/before_stream.tar`，不覆盖其他工作区改动。启动说明见 `MISSION_FRAMEWORK.md`。
- 独立 CLI 另做 3 秒启动/SIGTERM 退出检查：日志出现节点就绪，外层 timeout 预期返回 124，相机释放，无相关后台 Python 进程；日志 `cli_shutdown.log`。当前节点未常驻，不占相机。完整机械/导航后端尚未存在，不启动真实任务调度去等待虚假动作反馈。

## 局部对准软件（2026-10-31 已部署）

- 新增 `alignment.py`、`alignment_demo.py`、`alignment_ros.py`、`tools/alignment_ros_smoke.py`、`tests/test_alignment.py`。流程为固定观察姿态请求→动作端确认停稳→等待稳定时间→新帧观测→限幅修正→停稳反馈→重新观测，容差内连续新帧且满足时间跨度才 aligned，来源过期后变 expired 并清空目标。当前仅支持原料区指定颜色，独立于整轮任务，不自动替换 STM32 主控或任务调度器。
- 标定绑定相机/原图尺寸/点定义/HSV 哈希/观察姿态/平面和本车局部像素误差到动作 X/Y 的 2×2 映射。真实模式缺标定拒绝 start，模拟标定与真实模式严格隔离；本轮没有生成本车真实标定或照抄截图比例。修正请求附来源及失效期限，实际动作端须在执行前检查期限和标定。默认 3 px、单步 10 mm 等仅为软件可配预算，未验证为本车可用值。
- 独立主题 `alignment[_sim]/control`、`vision_command/feedback`、`motion_command/feedback`、`status`；视觉子协议兼容现有后端，未来通过重映射复用唯一相机，不另开采集。动作端接口 `hold_observation_pose/adjust_xy/stop` 仍待真实驱动。丢目标/歧义/标定不匹配/超时/预算耗尽请求停止；停止未确认保留故障且禁止重新开始，ACK 不等于完成，不盲目重发修正。没有通过对准模块生成抓放成功计数。
- Mac 全部视觉范围测试 `204 passed, 172 subtests passed`；Pi `203 passed, 1 skipped`，跳过仍为网页 JS 模拟。测试包括真实视觉适配器协议联通（假数据，无硬件）、旧帧/高帧率连续确认、修正限幅、累计/次数预算、停稳后帧下限、相机会话变化和停止未确认阻止重启。
- Pi ROS 2 真正 pub/sub 的纯模拟正常例：2 次修正后 aligned，stop_confirmed=true；target_lost、motion_timeout 均 fault 并确认停止；stop_timeout 为 fault、stop_confirmed=false。证据 `output/material_diagnostics/alignment_ros_20261031_UyW7Yq/` 下 `normal.json`、`target_lost.json`、`motion_timeout.json`、`stop_timeout.json` 及日志。使用独立 domain 75、LOCALHOST，仅模拟主题，无相机/串口/电机/雷达。结束后无相关后台 Python 进程或相机占用。
- 说明见 `ALIGNMENT_INTERFACE.md`。此阶段证明软件流程和 ROS 收发，不代表本车运动收敛、抓放精度或整车通过。下一步先补位置身份/双摄角色与电控联调所需接口，真实动作等待观察姿态、轴方向和标定确认，不安排反复人工摆放作为软件门槛。

## 2026-10-31 视觉检查记录（历史，不作为框架开发门槛）

### 本轮 ROS 状态核对与论文参考

- 用户确认另一个对话已部署 ROS 2。本轮重新读取交接和 `MISSION_FRAMEWORK.md`：已完成的是任务调度及 ROS 主题模拟，不是已接真实相机的视觉后端。Pi 只读检查确认 `mission_ros.py` 存在、稳定相机入口存在，未发现相关运行节点；未重启、打开相机或串口，也未重复模拟测试。设备入口存在不证明实际采集健康。
- 当前建议下一项为按请求接入真实视觉结果后端：复用现有采集/HSV 与时效逻辑，先接 `read_task`、原料区 `scan_material`；不依赖网页。粗加工重抓需要 position_id，`locate_position` 需要真实位置身份，尚不支持时明确返回未支持/失败，不填固定位置。按“固定路线粗到位→朝向校正→固定观察姿态→视觉对准→重新观测→标定动作”准备接口，但任务流程所有者仍须与电控确定，不能默认树莓派接管 STM32 主流程。
- 论文 1：Bi 等，ICAROB2024，*Intelligent Logistics Handling Robot: Design, Control, and Recognition*，DOI `10.5954/ICAROB.2024.OS13-1`，来源 `https://alife-robotics.co.jp/members2024/icarob/data/html/data/OS/OS13-1.pdf`。网站直取多次超时，已通过同一原站检索返回的第 6 节相关正文核对：6.1 用 HSV 二值化、膨胀及轮廓获得物料图像重心；6.2 用二值化、轮廓筛选、最小外接圆定位色环，并非直接霍夫圆。未获得可直接移植的六色阈值/本车精度证据；不声称已完整核验所有页或重现实验。
- 论文 2：杨玉燕、高洪，《基于仿真工业场景的智能物流小车设计与实现》，DOI `10.61369/TACS.2025090014`，来源 `https://api.arttechpress.com/uploads/file/asp/2026011913541686b3d3799.pdf`，共 3 页。第 2—3 页采用编码器路径闭环、陀螺仪朝向校正、视觉 X/Y 对准；物料是红绿蓝三色 HSV＋轮廓，色环使用 YOLOv8＋OpenVINO。不是六色阈值验证，且其 i7 视觉主机性能不能类推 Pi；本文不足以证明本车需要切换 YOLO。
- 可借鉴：保留分色逻辑，用既有图像离线比较框中心、图像矩重心、圆/椭圆候选；轮廓形态处理需检验粘连和反光偏移。当前输出仍是框中心，未修改算法或默认参数。现有 RingDetector 已有霍夫/轮廓最小外接圆与椭圆候选，真正缺口还包括位置 1/2/3 身份和真实标定；斜视椭圆中心不自动等于物理圆心投影，须用本车平面标定验证。算法候选不能在未评估前升级为抓放坐标。

- 用户确认先完成 Ubuntu 视觉，并询问地图排期。顺序仍为必要的视觉实物确认 → ROS 2 视觉接口；随后可开展初赛名义地图、车道约束和软件路线验证。实际定位与整车路线验收依赖底盘尺寸、运动反馈及真实场地，不必等待机械臂全部完成。本轮未创建地图、接入导航或操作雷达。
- 用户计划使用两台 USB 相机：一台扫码，一台安装在机械部分向下看物料。此为计划分工，双摄同时运行尚未验收；移动相机的抓取坐标需结合观察姿态/高度标定。行驶时地面边界是否进入相机视野尚未确认，不能直接认为两个视角已经覆盖车道检测。
- SSH 别名 `pi-vision` 实际连接成功；检查时未运行视觉服务，8090 无监听。系统只列出一台 Generic USB Camera；`video-index0`、`video-index1` 是同一台相机的两个设备入口，不是两台相机。
- 本轮 Mac 视觉测试 `130 passed, 89 subtests passed`，Pi `130 passed`。确认相机无占用、扫码副相机配置为空、相机控制配置为空后，恢复文中既有 HSV 物料基线命令；未打开串口，未更改颜色参数。运行目录 `/home/cuiguo/competition_vision/output/material_diagnostics/ubuntu_resume_20261031_V8sTfa/`，日志 `service.log`，检查时视觉 PID 1677；进程号仍仅为快照。
- Mac 本地 8090 隧道已恢复，`/state` 采集/物料状态有效、错误计数 0，`/frame.jpg` 可读。这只确认当时在线。实际预览朝向天花板并有局部遮挡，不是物料验收场景，接口中识别出的颜色不能作为正确检出证据。已请用户把当前相机临时朝向桌面、让一件物料完整进入识别框，再做移出/恢复；Ubuntu 扫码复验、双摄、ROS 2 仍未完成。
- 用户随后报告预览反复黑屏、提示画面过期。检查时 Pi 采集持续更新、相机/物料错误计数为 0；Mac 经隧道取图 25 次全部收到 JPEG，但耗时 343.8—1330.8 ms，按旧网页“服务端帧龄＋完整请求耗时 < 1000 ms”规则，12/25 次会被丢弃。这解释当时正常采集但网页闪黑的现象，不代表已排除所有网络或设备故障。
- 预览修正：新增仅人工看图的 `--preview-timeout`（默认 3 秒、HTTP 头 `X-Preview-Max-Age-Ms`），不更改采集/识别默认 1 秒有效期及服务端失效图像 503。网页超过采集期限时明确标注延迟、隐藏图上检测标记；短暂传输失败可在原显示期限内保留上一帧并提示，不续期，超出显示期限仍清空；隐藏页面即清空。新 JS 的确定性模拟覆盖慢传输、标记失效、显示期限、短暂失败、恢复、旧服务兼容、缺失年龄、页面生命周期、重连在途结果丢弃及连续 2.2 秒慢请求；Mac 全部测试为 `133 passed, 103 subtests passed`。Pi `132 passed, 1 skipped`，跳过项为缺少 Node.js 的 JS 模拟，已在 Mac 执行，不为此在 Pi 安装运行时。
- 实际部署后 3 秒显示期限下遇到连续约 2.2 秒取图，按保守时序仍有过期间隙；当前网络运行参数改为 `--preview-timeout 5`，识别和采集仍为 1 秒、原图/ROI/HSV/预览大小及质量均不变。最终服务检查时 PID 3459，日志 `/home/cuiguo/competition_vision/output/material_diagnostics/preview_network_final_20261031_YDIfJW/service.log`，唯一持有相机，未打开串口。重启后取样启动阶段包含连接重置/首帧等待 503，随后 12 次 JPEG 成功，耗时 481.8—1518.1 ms；HTTP 头显示看图 5000 ms、采集 1000 ms，这 12 次按请求时序无显示过期间隙。旧代码备份在 `backups/preview_network_20261031_YHnD5L/before_preview.tar`。
- 实际浏览器的自动读取通道不可用；用户随后回复“ok了 我们继续”，记录为用户确认本次预览恢复，不要求再次刷新。网络仍有延迟，不能用本次确认、JS 模拟或 HTTP 正常替代长时间运行确认；物料/扫码实物验收仍未完成。
- 续查实物画面：桌面有红/黄/蓝，原 ROI `(480,60,520,450)` 未完整包含左侧物料，且包含键盘；首次当前观测有红色、两个蓝色结果，黄色未检出，额外蓝色位置对应键盘附近，不能据此宣称三色通过。未修改 HSV/ROI 或重启服务。随后红色被移到 ROI 外，当前有效结果无红色；未记录移动瞬间，故不能量化消失响应时间。5 秒诊断保存在 Pi `output/material_diagnostics/ubuntu_presence_20261031_WrHY0O/before_removal.json`，25 次轮询中有 19 个有效独立来源、1 次无效、5 次重复；实际采样时红色已在框外，该文件名不表示“移出前静止基线”。此轮还含黄色缺失/额外颜色，不作为分色正确率验收。
- 上述过程中采集累计 78 次读帧错误、4 次重开、epoch=1，随后采集与物料结果恢复有效；日志有 `camera_read_failed`、V4L2 errno 19 `No such device`、相机未打开；内核 18:55:14 记录 USB Camera 断开及重新枚举，具体原因尚未确认。不能与此前网页网络延迟混为一谈，也不把这次非受控中断当成 USB 故障恢复验收通过。
- 用户确认红色放回后，读取现有服务进行 5 秒检查：25 次轮询、25 个新鲜独立观测，红色每次恰好一个，25/25；同一服务会话，采集错误计数保持 78、重开计数保持 4，期间未新增中断。报告在 Pi `output/material_diagnostics/ubuntu_red_return_20261031_T36ACd/red_return.json`。结合上轮框外无红色，此次仅确认该场景下红色移出后的清空、放回后的重新检出，未测动作瞬间的响应延迟，也不代表六色或整体视觉通过。新画面红色仍被原 ROI 左边界截到，坐标不能作完整轮廓定位或抓取精度证据；没有为此反复让用户摆放或更改颜色参数。报告中黄/蓝在背景且不完整，不作为三色验收。
- 下一步为 Ubuntu 扫码功能检查：先让用户准备合法任务二维码，再切换当前唯一采集服务到扫码模式，避免二维码 10 秒确认窗口在准备时已耗尽。当前扫码仍关闭，不启动第二个相机采集程序。项目测试图 `examples/qr/task_156.png` 已在 Mac 解码为合法候选 `156+123+516+231`；仅此图像解码不等于 Pi 实机扫码通过。
- 用户随后确认二维码准备好，但续查发现相机再次断流：内核 19:11:24 记录同一 USB Camera 断开及重新枚举，随后有 UVC probe/control `-71`、USB suspend `-32` 及 V4L2 select 超时。旧服务采集无效，物料输出为空；不能算扫码失败或成功。已核对原进程命令无串口后有序停止 PID 3459，并确认设备无占用、8090 已释放，再尝试同一相机的扫码专用启动（`--algorithms-on --no-materials --no-rings`、原 HSV/时效/预览参数不变）。
- 本次扫码启动 PID 4975 因 `无法打开相机` 退出，尚未获取任何扫码观测；日志 `/home/cuiguo/competition_vision/output/material_diagnostics/ubuntu_qr_scan_20261031_8vxQUy/service.log`。同目录 `state_after_scan.json` 是失败取状态留下的空文件，不是有效报告。设备入口仍存在，但不能据此认为相机可采集；检查时无视觉进程或相机占用，网页服务当前未运行。未改源码、驱动、USB 电源策略或雷达。下一步需用户只将相机 USB 拔下等 5 秒再插紧（不拔 Pi 电源或雷达），随后助手先核对设备与采集，再重启扫码；连接、供电或相机本体等具体原因尚未定位。
- 用户重新插接后，19:14:54 内核重新枚举同一 USB Camera；独占、有总时限的采集诊断成功读取 8 帧 1280×720，约 1.506 秒，随后释放相机。仅确认该次采集恢复，不认定掉线根因已解决。
- 用户明确要求“直接测试二维码，不再搞显示”。后续当前验收优先用独立命令行程序，不自动启动网页、推流或显示界面。确认相机无占用后直接运行 `pi_vision.qr_check`，同一稳定设备路径、10 秒扫码时限、至少 3 帧且最小确认跨度 0.2 秒，总进程外层 20 秒有界；不启动串口、YOLO 或雷达。
- Ubuntu 合法任务码实机确认成功：读取 5 帧，前 2 帧未找到码，随后连续 3 帧合法候选确认 `156+123+516+231`；程序 `status=ok`、`confirmed=true`、耗时 1.991 秒、退出码 0。证据目录 Pi `output/material_diagnostics/direct_qr_20261031_gLprGr/`，`result.json` 是结果，`observations.log` 是逐帧诊断。程序结束时在 finally 释放相机，未启动网页服务。此次仅确认该场景的合法码读取与连续确认；无码/非法码/重扫实物测试、长期 USB 稳定性及完整 Ubuntu 视觉仍待验，不提前开始 ROS 2 或宣称六色/定位通过。下一步可让用户拿走二维码后直接测无码超时，不能把之前确认的任务码沿用成本次扫码成功。
- 用户拿走二维码后，独立命令行无码实测完成：10 秒扫码窗口、读取 22 帧，结果 `status=timeout`、`confirmed=false`、`raw=null`、退出码 1（本次预期的阴性结果，不是采集错误），未沿用旧任务码。证据目录 Pi `output/material_diagnostics/direct_qr_absent_20261031_AlMTEs/`，`result.json` 和 `observations.log`。仍不启动网页或串口。合法码与无码两项分别有实机证据；非法任务码实物拒绝、重扫切换、长期运行及完整视觉仍待验。
- 用户提供朋友的坐标映射截图作为接口参考：图中树莓派返回圆心/颜色，STM32 `transfer_position_sehuan()` 以目标像素 `(955,519)` 和 `0.29239766` 的比例换算，另有绿色色环基准和 150 mm 间距。原项目分辨率、完整代码、坐标正方向、接收格式及这些数值对本车的适用性均未确认，不将截图内容当作修改/安装授权或标定事实；未照抄参数、未修改算法或通信协议。当前实机图像为 1280×720，现有 HSV 返回轮廓框中心而非拟合圆心，更非机械抓取点。
- 对接建议待与电控确认：树莓派输出原图像素（左上原点、右/下为正）和固定六色业务编号，电控侧负责一次像素→运动量转换，双方先统一点定义、分辨率、轴方向、单位及帧龄/有效性/丢目标处理；旧串口并没有完整时间和有效性字段，尚未宣称建议已实现或已联通。相机观察姿态、高度及目标平面固定后再标定参考像素和 X/Y 比例；视角倾斜等不满足简单比例时需检验更完整的平面映射，并考虑物料顶面高度。用户随后明确朋友截图仅供参考，不以索要其代码作为框架开发前置条件，也不凭截图改坐标语义。

## 电控方案与开源参考（2026-10-31）

- 用户提供参考仓库 `https://github.com/yuanxiexie531-glitch/gcs-gold-medal`，说明电控计划采用“固定路线粗到位＋陀螺仪校正朝向＋相机局部对准＋标定好的抓放动作”。记录为拟采用方向，不代表底盘、反馈或机械动作已实现。视觉与 ROS 2 推进顺序保持；本轮不操作雷达。
- 已通过 GitHub 只读核对仓库快照 `27a6e00fa227f78a001e0644d7f2272d045f94a7` 的 README、Python 视觉程序、STM32 `USER/main.c`、`can_ZDT/circle_pid.c` 与 `yyb_move.c`。参考实现包含分阶段视觉定标、主控像素差换算、角度 PID、预设抓放动作；`transfer_position_sehuan()` 确有此前截图中的参考像素 `(955,519)` 和比例 `0.29239766`。不能因此确认朋友截图必然出自同一版本，更不能将该参数作为本车标定。
- 参考代码含红/绿/蓝三色约定和两组三位任务码路径，与本项目六色业务编号、四组码语义不同；`wait_send_shumeipai()` 的等待循环没有实施传入的总超时，个别识别失败分支还有默认位移。只参考组织方式，不直接替换本项目算法、协议或失败语义。仓库所附旧赛事资料不替代本项目已核对的规则；在线方案说明文档尚未读取，本轮不声称完成整个项目审计。
- 对接建议：主控掌握任务与动作状态，粗到位并固定观察姿态后请求指定颜色/位置观测；视觉返回原图像素、目标身份、来源帧和有效性；主控按本车标定转换一次，限幅调整，停车后重新观测，连续新帧满足机械容差才允许动作。总超时、目标丢失或过期时停止本次对准并返回失败，不用旧点或默认位移继续抓放。底盘与机械臂谁执行修正、消息格式及阈值仍待双方确定。
- 陀螺仪朝向校正不消除平移误差，粗到位必须保证目标进入实际视野；移动相机需固定可重复观察姿态或结合姿态标定。物料顶面、地面圆环、码垛上层的高度差分别验证。固定路线仍须覆盖适用赛题的车道与随机障碍要求，不能据此宣布导航或避障完成。
- 参考核对时 Pi 只读检查 SSH 可连接、稳定相机入口存在，未发现 `pi_vision`/`qr_check` 运行进程；未打开相机、串口或启动网页。这不证明相机采集或 USB 长期稳定。此前计划先补齐 Ubuntu 必要功能，现按用户最新要求直接做任务框架；与电控仍需统一观察姿态、坐标/单位、对准完成和动作反馈接口。

## 比赛要求（依据本次附件核对）

### 来源与适用范围

当前项目对应“智能搬运”赛项，不套用同附件中的“智能分拣”或“智能救援”规则。2026-10-23 核对了以下三份用户附件，并在本地 `output/competition_rules/` 留存原文副本；该目录被 Git 忽略，不会随仓库克隆带走。下文页码均从 PDF 第 1 页开始计数。

| 来源代号 | 附件及本地副本 | 本次主要核对页 |
| --- | --- | --- |
| T | 《02-2027年第十届 智能+工程创新赛道竞赛命题与运行(3).pdf》；`output/competition_rules/tasks_national_2027.pdf`，标题为全国第十届 | 1—9 |
| A | 《2027智能+工程创新赛道竞赛命题解析(1).pdf》；`output/competition_rules/task_analysis_2027.pdf` | 9—25 |
| S | 《智能+工程创新赛道评分与规则(1).pdf》；`output/competition_rules/scoring_jiangsu_2027.pdf`，标题为 2027 年第九届江苏省赛 | 1—4 |

**待确认：用户实际参加的比赛层级、后续补充通知及 S 是否为本场正式评分依据。** 下列 S 分值和时限是该附件明文，不能无条件当作全国赛最终规则。若不同来源冲突，记录差异并以适用赛事正式文件/裁判确认为准，不自行合并。这里只整理与本项目直接有关的智能搬运要求。

### 初赛任务、场景与视觉输入

- 自主完成：读任务码并显示 → 第一批按顺序从原料区抓取 → 粗加工区按指定位置放置 → 按顺序再次抓取运至暂存区平面放置 → 第二批重复原料区和粗加工区流程 → 暂存区同颜色码垛 → 返回抽签启停区，显示正确抓取/放置次数。（T 7—8；A 19—23）
- 每次抓取一个，放到车上后才能抓下一个；跨区域运送时物料须装载在车上，不能用手爪一直夹持搬运。车可携带 1—3 件运送。（T 7—8）
- 初赛六色编号：红 1、黄 2、蓝 3、绿 4、黑 5、浅蓝 6；每种颜色两件，抽取三种不同颜色，两批颜色相同。物料为 3D 打印 ABS/PLA，尺寸不超过 80 mm、质量不超过 100 g，浅蓝为 PLA，其余五色为 ABS；初赛三种颜色采用同一种形状，现场允许一定色差。（T 5—6；A 12、14）
- 场地 2400×2400 mm，灰色车道，中间十字车道宽 400 mm；启停区 300×300 mm；粗加工区和暂存区各 580×150 mm。区域和二维码板位置存在现场偏差，不能依赖固定相机像素位置作为比赛解法。（T 3—4）
- 避障是智能搬运功能要求；赛道设黑色模拟障碍物，尺寸 φ50×100 mm，数量和位置随机抽签放置。规则未指定必须使用激光雷达，但不能因没有雷达专项分而省略避障。仅停车保护不等于完成受车道约束的避障任务。（T 1、3；A 9）
- 转盘顶面直径 300 mm、高 80—100 mm，三件物料均布、相隔 120°；转速 6—10 秒/圈、转向随机，停留次数/时间现场确定，停车点有误差。不能假定固定角速度方向、固定等待时间或精确停车位置。（T 4；A 12）
- 二维码板为横向 A4、垂直安装，码居中且含边框 80×80 mm，板的位置在规定范围内随机。（T 6；A 16）
- 车上部醒目处须有明亮、无遮挡的任务码显示装置，字高至少 12 mm，并能显示任务码与完成情况；解析说明要求比赛中始终显示任务码。网页调试预览不能代替车载显示。（T 2；A 10）

### 四组任务码必须保留的语义

| 组 | 含义 |
| --- | --- |
| 第 1 组 CCC | 第一批三个物料的颜色和搬运顺序 |
| 第 2 组 PPP | 第一批在粗加工区和暂存区的放置位置 |
| 第 3 组 CCC | 第二批三个物料的颜色和搬运顺序 |
| 第 4 组 PPP | 第二批在粗加工区的放置位置 |

四组三位数以 `+` 连接。第二批在暂存区须放在第一批**同颜色**物料上，且第一批须放置正确、上层稳定不掉落；因此第二批暂存位置由第一批颜色对应位置确定，不由第 4 组直接决定。（T 6—8；A 15、22；S 3）

示例 `156+123+516+231`：第一批红/黑/浅蓝依次放位置 1/2/3；第二批黑/红/浅蓝在粗加工区依次放位置 2/3/1，到暂存区则分别码垛到第一批黑/红/浅蓝所在位置 2/1/3。（前半为 T 6 示例，暂存位置为按码垛规则推导。）

### 放置精度与评分（S 适用性待确认）

**放置点的数字 1/2/3 与一个放置点内由内到外的评分同心环 1—6 是不同概念。** 视觉应同时解决“哪个位置”和“目标中心在哪里”；已有 `ring_no=1` 不能完成位置编号识别。（T 4—6；A 13、20—21）

| 评分同心环 | 1 | 2 | 3 | 4 | 5 | 6 | 6 环外或倾倒 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 分值/次 | 15 | 10 | 7 | 5 | 3 | 1 | 0 |
| 外径/mm，φ 为物料最大直径 | φ+3 | φ+8 | φ+15 | φ+25 | φ+35 | φ+45 | — |

外径由 T 5 表 3 的逐级增量换算，线宽 1.5 mm（T 4）。图示不等于可直接采用的抓取误差阈值，须结合真实物料底面、安装高度及测量方式验证。

- 平面放置必须按任务码顺序和位置垂直放到对应圆环；按底面处可见的圆环边缘判环，不是只检查检测框中心是否落进某圈。颜色/顺序/位置错误不能靠圆心准确补救。（S 2—3；A 20—21）
- 底面接触地面即判本次放置完成，不能推动物料修正位置；S 3 同时有“若再次移动此物料，本轮比赛结束”的表述。T 7—8、A 22 又明确要求粗加工放完后重新抓取搬运；**待向裁判确认这条对纠偏移动和正常后续抓取的具体边界，不能自行取消规定的再抓取流程。**
- 第二批暂存码垛必须下层正确、同颜色、不掉落，得分同相应第一批物料的放置分数，不影响第一批既有成绩。（S 3）
- 正确扫码且在装置显示：1 分；按任务顺序正确抓取并放到车上：每次 2 分；完成全部流程回到指定启停区：2 分；最终正确显示抓取数量、放置数量各 2 分，无相应动作不得分。（S 2—3）
- 初赛总成绩 `P=20%A+10%B+70%C`，A 为任务命题文档、B 为作品创意设计、C 为现场初赛；S 中现场 C 按本队原始得分/该场参赛队最高得分×100 归一化。两轮运行取最好成绩；不能把一项 15 分当成归一化现场总分。（T 8；S 1、4）
- 决赛创新实践/现场决赛权重为 30%/70%，有 A/B/C 等级约束，具体任务、物料形状/颜色、编码获取方式和评分现场公布；不能把初赛六色固定方案宣称为覆盖决赛。（T 6、8—9；S 4）

### 运行约束与设计含义

- S 3：每轮调试 3 分钟、运行 3 分钟；T 7：调试剩 1 分钟抽任务，剩 30 秒停止场地运行并准备出发。统一指令后一次一键启动，开始后不得再触碰车、笔记本或物料，不允许遥控。（T 2、7；S 3）
- S 3 对停机限制原文为“停止运行 15 秒（等待转盘转动时间增加 8 秒）”；A 25 简述为 15 秒。**额外 8 秒的裁判认定与适用范围待确认**，不能把所有识别/重连等待统一设成 23 秒。
- 完全离开启停区后在灰色车道行驶；S 3 明确车体铅垂投影越出车道即结束，机械臂除外，启停区除外。避障路径须满足车体占据范围与车道约束；雷达扫描平面不能代替地面颜色边界判断，需结合实际导航方式验证。（T 3；S 3）
- 只允许垂直向下补光，不许遮挡场地；颜色算法须适应现场光照和物料色差。传感器种类和数量不限，文件未要求必须使用某种开源模型、YOLO 或 ROS 2；HSV/ROS 2 属于我们的技术选择。（T 1—2、5；A 9）
- 整机含软线出发前铅垂投影不超过 300×300 mm，高不超过 400 mm，折叠机械臂出发后才可展开；单个内置电源，比赛含调试不更换电源或零部件。相机、补光与线缆安装要考虑这些约束。（T 2）
- 工程推论：视觉结果应及时提供给主控，自主处理超时与失效，不依赖赛中网页点按、SSH 人工重启或手动改任务码；正确抓取/放置计数须基于实际闭环而非“发了坐标即成功”。这些是由上述规则导出的设计目标，不是额外官方条文。

### 与当前实现的对应

- 已有：任务码严格确认、六色 HSV 基线、全图像素中心与状态时效信息。仍需新系统实机复验。
- 未完成：位置 1/2/3 识别、抓取/放置坐标标定、转盘实机门控、第一批颜色到暂存位置映射的主控联调、第二批同色码垛定位、实际成功检测与车载显示统计闭环、三分钟整轮运行。
- ROS 接口需承载批次/目标颜色/目标位置等必要任务信息，避免丢失四组码的语义；具体分工和消息结构待与主控确定。95% 检出率、3 px 抖动等仍是工程验收目标，附件未规定这些指标。

## 路径与版本

- Mac 工程：`/Users/jzxzhou/Documents/ChatGPT/车子`。
- 当前树莓派工程：`/home/cuiguo/competition_vision`；用户、主机名均为 `cuiguo`。
- 私有仓库：`https://github.com/chichoo103322/competition-vision`，远程名 `origin`，分支 `main`。
- Mac 与 Pi 的 HEAD 均为 `f563961`，但已有同步到 Pi 的视觉参数/诊断接口工作区改动，尚未提交或推送。交接文件创建在 Mac，并已同步到 Pi；原文件见 Pi `backups/hsv_interfaces_20261023/original_sources.tar`，新文件不在此历史备份内。
- 原 `/home/j/competition_vision` 和用户 `j` 属于重刷前环境，不再用于新系统命令。
- 本次整理前 Mac 有未跟踪文件 `tools/pi_network_diagnose.sh`；保持原样。它使用 NetworkManager/nmcli，不能直接作为当前 Ubuntu 网络的通用诊断入口。
- `docs/` 是被 Git 忽略的本地历史资料，部分内容过时；不要依赖其中旧账号、旧地址、旧测试数量或旧后端结论。

## 已确认环境与连接

- Raspberry Pi 5 Model B，约 4 GB RAM、58 GB 根分区；Ubuntu 24.04.5 LTS arm64，内核 `6.8.0-1065-raspi`。
- 系统 Python `3.12.3`，发行版 OpenCV `4.6.0`；cv2、numpy、yaml、serial 可导入。
- 已安装 ROS 2 Jazzy，`/opt/ros/jazzy` 存在，加载环境后 `rclpy` 导入成功；本项目任务节点与单相机视觉后端已实现，均以 Python 模块运行，尚未封装独立 ROS 包。
- `.bashrc` 未发现 Jazzy 环境加载项；新树莓派终端需要手动 `source /opt/ros/jazzy/setup.bash`。
- 用户已加入 `video` 组，USB 相机可列举。视频采集设备当前为 `/dev/video0`，`/dev/video1` 不作为当前采集入口；`video19`、`video20…37` 是平台设备。
- 相机稳定路径：`/dev/v4l/by-id/usb-Generic_USB_Camera_200901010001-video-index0`。
- 公共 Wi-Fi 为 XBOTPARK，无路由器管理权、无网线/网口转接器；SSH 密钥已配置，无须反复输入登录密码。
- Mac 的 `~/.ssh/config` 已有 `pi-vision` 别名，用户 `cuiguo`，密钥 `~/.ssh/pi_vision_ed25519`，HostKeyAlias 为 `cuiguo.local`。
- 别名连接 `fe80::2ecf:67ff:fe6b:583c%en0`（SSH 配置 HostName 中 `%` 写成 `%%`）；当前 AddressFamily 为 `any`，不强制 `inet6`，避免本地 IPv4 隧道无法绑定。
- 已保留主机身份校验。确认过的 ED25519 指纹：`SHA256:HHSX+pn2T+/SF2vgjggOl43qfWxcswg5/wyhI+z9VSY`。
- 检查时 Pi IPv4 为 `172.16.25.114`；IPv4 会变化，不能固定依赖。`.local` 名称解析曾间歇失败，公共网络策略是可能因素，具体网络侧原因未证实。
- IPv6 别名和密钥登录已连续验证成功；链路本地地址仍要求两设备可在同一链路互通、Mac Wi-Fi 使用 `en0`。不保证换网络、换网卡或公共网络隔离后仍可用；未做断电/多次重启稳定性验收。

## 雷达接入状态（2026-11-01 更新；早期记录标注日期）

- 用户提供 YDLIDAR T-mini Plus 底层协议手册并明确开始雷达开发，随后确认已插入树莓派。USB 已识别 `10c4:ea60 Silicon Labs CP210x UART Bridge`，内核驱动 `cp210x`，设备 `/dev/ttyUSB0`；这仅确认转接板，不代表已读到雷达型号或有效扫描。
- 稳定设备路径：`/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`。检查时未发现当前用户的雷达进程/串口持有者；`ModemManager` 在运行，是否干扰雷达尚未验证，不为此先行停服务。
- 早期串口权限问题已解决：2026-11-01 新 SSH 会话确认 `cuiguo` 属于 `dialout`，对稳定设备路径可读写；无需重复加组或重启。旧 SSH 复用会话可能保留旧组，检查使用 `ssh -S none pi-vision`。
- 已从厂商官方仓库取得 YDLidar-SDK 1.2.20、源码快照 `42a82ed`，在 Pi `/home/cuiguo/ydlidar_work/` 独立编译官方 `tmini_test`，可执行文件为 `/home/cuiguo/ydlidar_work/build/tmini_test`；编译成功、ARM64 动态依赖均可解析，没有执行系统安装。归档 SHA256：`257e9bc39fafae05e9f9c2f290c9940bcf3c1676111c9226527d164af9c7e0df`。保留厂商源码与校验逻辑，未因历史 issue 关闭数据校验。
- 雷达型号和 SDK 连续扫描已于 2026-11-01 确认，见下；ROS 雷达驱动未安装。启动任何新采集前仍须核对串口占用，使用明确 by-id 路径，不打开电控串口。实际扫描高度能否照到高 100 mm 的比赛黑色障碍物、黑色检出与距离精度仍待实测。
- 2026-11-01 用户执行 `sudo usermod -aG dialout cuiguo` 后，新 SSH 会话已确认 `cuiguo` 具备 `/dev/ttyUSB0` 读写权限，串口检查时无进程占用。树莓派官方 SDK `tmini_test` 已有界连接并返回健康正常、型号 T-mini Plus/151、固件 1.2、硬件 1、序列号 `2025011000090206`；检查时先读到 6 Hz，测试程序随后调到 10 Hz 并退出，退出后无残留进程或串口占用。日志中有一次 `Fail to get pitch angle`，不等同于二维距离扫描失败；连续有效点云、安装水平、障碍物检出、原始数据留存和 ROS 2 接入仍未验收。此次只确认设备链路，不实现舵机、栅格或 A*。
- 同日限时 12 秒 SDK 扫描复核：输出 88 个扫描摘要，每帧 430 点，数据失败 0 次；启动过渡阶段频率读数不稳定，末段稳定约 10 Hz。外层 timeout 返回 124 是有界测试预期，SDK 已发送停止命令，结束后串口无占用。证据目录为 Pi `output/material_diagnostics/lidar_link_20261101_190705/`，其中 `summary.json` 和 `sdk_scan.log`；只证明持续点云摘要与正常停止，不证明距离真值、障碍物检出、安装高度或建图精度。
- 2026-10-23 雷达检查未重启当时的视觉服务；此处不代表视觉服务当前在线，最新相机快照见后文。

## 代码架构与关键决策

- `pi_vision/camera.py`：OpenCV VideoCapture / V4L2 采集，可选去畸变。
- `pipeline.py`：组合二维码、物料、圆环检测，`material_region.py` 限定 ROI 并还原全图坐标。
- `stream.py`：采集和检测线程、HTTP 预览及 `/state` 状态，带可选串口桥；预览减负不改变检测原图。`preview_page.py` 是网页 UI。
- `observation.py`：stream 路径的来源时间、观测有效性和过期/故障状态；`vision_ros.py` 经同一采集模块复用其语义，`vision_backend.py` 适配任务反馈，不另开相机或复制检测器。
- `task_confirmation.py`：严格任务码校验与连续确认；`material_tracking.py`：短暂丢失的有时限坐标保留；`rotation_gate.py`：转盘停稳门控。
- `app.py`、`state_machine.py`、`protocol.py`、`serial_link.py`：原有任务控制与串口路径，尚需正式闭环验收。
- `world_map.py`、`plane_calib.py`：像素到平面的标定工具；当前 Pi 配置内相机矩阵、畸变系数、平面单应矩阵均为空。
- `robot_perception/`：独立任务码/颜色等工具；`lidar.py` 只有基于输入扫描的停车判断逻辑，没有雷达硬件驱动或 ROS 扫描接入，默认阈值没有实车验证。
- 业务颜色：1 红、2 黄、3 蓝、4 绿、5 黑、6 浅蓝。
- 任务码：`CCC+PPP+CCC+PPP`；颜色 1—6，两批为相同的三种不同颜色；位置组各为 123 的排列。默认确认至少 3 个新帧且跨度至少 0.2 秒，断码/换码/间隔过长重新累计；具体入口可覆盖参数。
- 当前选用显式 `--material-backend hsv`，实现为 `HSVMaterialDetector`；并未把源码默认 `auto` 改成 HSV。`auto` 可能选择旧 YOLO 或另一颜色检测器，不可省略此参数替代现基线。
- 原模型由设备原有工程提供，训练者未知。历史训练审计发现语义错标；模型与具体训练版本的对应关系未建立，不能简单换类别名称或降低阈值修复。当前没有新训练并验收的权重。
- HSV 已修复蓝/浅蓝重复或相接区域问题：组件颜色判定、封闭高光/暗斑归属外轮廓、相接颜色核心 watershed 分区；不是通用实例分割。
- ROI 为原图 `(x,y,w,h)=(480,60,520,450)`；原图 1280×720，预览宽 640、5 FPS、JPEG 质量 55。该 ROI 依赖现场摆放，重装相机需重验。
- 当前基线关闭二维码、圆环、转盘门控及去畸变；不自动打开串口。相机配置 FPS 是请求值，不等于已测实际采集 FPS。
- 现场参数接口：`--hsv-profile <YAML>`（必须配合显式 `--material-backend hsv`），模板 `config/hsv_profile.example.yaml`。支持分色 HSV 范围、面积/形状筛选、蓝/浅蓝分区阈值，缺省保持原值；参数合法性校验、`/state` 参数 SHA256，重启生效，无在线调参接口。本机配置继续 Git 忽略。ROI 保留原 CLI；未来 ROS 节点可复用 `VisionPipeline(..., hsv_profile=...)`，不代表节点已实现。
- 可选主相机控制：本机配置 `camera.v4l2_controls`，默认 `{}` 不改变设备状态。支持曝光/白平衡自动开关与对应值、增益、工频；数值范围/单位依设备查询，失败终止启动并释放设备。尚未在本机实际应用非空控制项，当前自动曝光/白平衡保持开启。
- 诊断接口：`--diagnostics-dir <新目录> --diagnostics-samples 30 --diagnostics-interval 1`，异步限量保存识别所用全图 PNG＋同帧结果/ROI/有效参数/来源帧号/Unix与单调采集时间；队列满时丢诊断样本不等待写盘，达到上限后视觉继续运行。`python3 -m tools.hsv_replay <目录> [--profile <候选YAML>]` 不占相机，可精确回放或比较候选参数。详见 README“现场参数与原图诊断”。

## 进度与已解决问题

- 二维码合法/非法/无码及连续确认此前已实测；Ubuntu 下二维码识别实机复验待做。
- 重刷前 HSV 黄/蓝/浅蓝 30 秒静止抽样：149 个新鲜独立观测，各色单目标 149/149、无多目标、无非预期颜色；黄色横向跨度 15 px，仍需定位评估。这是同场景抽样，不代表全部帧、六色泛化或抓取精度。
- Ubuntu 迁移后，用户运行视觉测试 `100 passed in 1.68s`；USB 相机已确认，源码已恢复。
- 当前视觉进程已运行；本次只读检查 `/state` 证实 HSV 物料检测开启、二维码/圆环关闭，采集原图 1280×720，采集/预览/识别状态存在新鲜时间信息。
- Mac SSH 隧道和网页首页已验证 HTTP 200；此前只监听 `::1`、浏览器访问 `127.0.0.1` 的冲突已修正，隧道显式绑定本地 IPv4。
- 尚未收到用户对 Ubuntu 下网页视频、六色识别及移动响应的人工确认；接口正常不能代替此项验收。
- 历史预览延迟/断流已做减负和时效处理，仍需新系统长时间运行统计，不能承诺彻底消失。
- 2026-10-23 续查：Mac 与 Pi 均在 `f563961`，两端视觉测试均为 `100 passed`；Pi 上仅一个 `pi_vision.stream` 进程持有稳定设备路径，运行参数仍为显式 HSV、1280×720、ROI `(480,60,520,450)`，状态帧新鲜；ROS 2 Jazzy 和 `rclpy` 可用。
- 2026-10-23 当前六件堆叠/遮挡场景静止抽样 30 秒，149 个新鲜独立观测：红/黄/蓝/浅蓝检出率均为 100%，绿为 51.68%，黑为 85.23%；绿中心跨度 11×22 px，黑为 158×79 px，未达当前静止基线目标。实时画面显示目标相互接触且存在遮挡，因此这不是合格的分组静止验收布置；原始记录在 Pi 的 `output/material_diagnostics/ubuntu_baseline_current_scene_2026-10-23.json`（Git 忽略）。Ubuntu 视觉基线仍未完成人工确认，按用户要求暂不开始 ROS 2 节点实现。
- 2026-10-23 用户调整第一组后，预览确认红/黄/蓝完整位于 ROI 内，执行 30 秒静止抽样：149 个新鲜独立观测；红 127/149（85.23%）、黄和蓝各 149/149，无重复目标或非预期颜色。中心 x/y 全距：红 2/7 px、黄 16/13 px、蓝 0/6 px。该组未通过 ≥95% 检出率及各轴 ≤3 px 的暂定工程目标；原因尚未确认，未改颜色逻辑。记录在 Pi 的 `output/material_diagnostics/ubuntu_red_yellow_blue_2026-10-23.json`（Git 忽略）。
- 2026-10-23 第二组绿/黑/浅蓝完整位于 ROI 内，30 秒抽样获得 149 个新鲜独立观测，三色均 149/149，无重复目标或非预期颜色；中心 x/y 全距依次为绿 24/6 px、黑 7/5 px、浅蓝 4/6 px，定位稳定性未通过。记录在 Pi 的 `output/material_diagnostics/ubuntu_green_black_lightblue_2026-10-23.json`（Git 忽略）。两组最大采集年龄约 101 ms、最大识别年龄约 271 ms；自动曝光/白平衡与抖动的因果尚未验证。
- 参数/诊断接口部署验证：Mac `108 passed, 82 subtests passed`，Pi `108 passed`；新服务单独持有相机，未打开串口，默认阈值和相机控制保持原状，显式加载等价基线样例。Pi 证据目录 `output/material_diagnostics/hsv_interfaces_H8YUneR7/`，原图在 `raw/`，30 张均为 1280×720，30/30 精确重现所记结果，改动前 HEAD 检测器对这 30 张图也全部输出相同结果。`acceptance.json` 为重启后 30 秒绿/黑/浅蓝抽样：各 149/149，中心跨度依次 24/6、4/3、0/3 px，有 1 个观测出现额外颜色，未通过正式基线验收。`green_contour_diagnosis.json` 显示绿色轮廓框由宽 161 px 扩到 208 px，左边界约不变，框中心跳变约 24 px；说明轮廓右侧扩张直接影响框中心，尚未确认反光/阈值/接触的具体成因，未替换定位策略。诊断已保存到 30 张上限，无丢弃/写盘错误，服务继续运行；该轮完整启动命令在 Pi 的进程参数中可核对，不能假定永久存活。
- 红/黄/蓝补采：Pi `output/material_diagnostics/red_baseline_H5SEXGDh/` 保存 60 张全分辨率原图及同帧结果，60/60 可精确回放、红色均被检出；同轮 30 秒状态抽样三色均 149/149，无重复目标或额外颜色。中心 x/y 跨度：红 21/12 px、黄 22/2 px、蓝 0/0 px。当前摆放未复现此前红色漏检，不能据此判定问题已修复；红色最大轮廓圆度范围约 0.43—0.74，高于默认 0.3 筛选阈值。黄色顶部接近 ROI 上边界，本轮主要作诊断，仍需正式安装条件下复验。阈值/相机控制未改变。当前服务使用此目录，诊断上限 60、间隔 0.5 秒；已达到上限仍继续识别。
- 后台进程历史问题：时效改动前的 `nohup ... &` 实例未响应 SIGINT，重复启动被相机占用拒绝，日志在 `output/material_diagnostics/red_baseline_X0zdyDsp/service.log`，不可混入有效样本。当前 stream 已显式恢复 SIGINT/SIGTERM 的有序停止处理；实机验证正常运行时 SIGTERM 后约 0.13 秒释放相机及端口。重启仍须核对进程、串口用途，并确认停止生效再启动；未新增持久服务管理器，原生调用永久卡住仍可能需要外部进程级重启。

## 时效与故障处理（2026-10-23 已部署）

- `/state` 为 schema 2，新增每次服务唯一 `session_id`、来源帧号、Unix/单调采集时间、处理完成时间、坐标系 `camera_image` 与像素单位。采集时间是主机读帧完成时刻，不是真实曝光时间；单调时钟用于失效判定，跨主机不得直接相减。
- 默认采集最大年龄 1 秒、结果源帧年龄/处理时限 1 秒、任务码本轮确认时限 10 秒；CLI 为 `--capture-timeout`、`--result-timeout`、`--qr-timeout`，有限正数、重启生效。相同来源帧只推理一次，不因轮询或重复处理刷新年龄。超过边界、处理错误、采集故障和未完成首帧均不能返回有效旧坐标。
- `*_observation` 返回 `status/valid/has_target`：`no_target` 是新鲜的成功空观测，`valid=true` 但无坐标；过期、检测超时、错误、采集失效、关闭均 `valid=false`。物料/圆环 getter 和 HTTP 输出同时清空无效坐标。采集故障恢复后须重新处理新帧，重扫/模式变化/捕获故障前的在途结果一律丢弃。串口桥不再沿用遮挡旧坐标，旧协议无有效性字段时统一 NOT_FOUND；此语义仅指当前 stream 桥，历史独立 app 路径尚未迁入。
- 严格扫码确认不变。`task` 为本轮锁定任务，可保留到停止/重扫；`qr` 实时几何位置过期即清空。Python `get_qr()` 的任务码可保留，点位仅在新鲜观测存在时返回，否则为 None。扫码会话超时、迟到解码、非法点位不得成功。任务保留不是持续可见或位置有效的证明。
- 读帧失败立即作废旧帧，连续失败由唯一采集线程释放/重开，退避间隔最多 5 秒；预览编码错误单独报告。检测超时不积压新任务、不接受迟到结果。原生读帧/推理永久阻塞无法在线程内安全强杀，状态会失效，但恢复可能需要外部进程级重启。网页空结果/故障/过期/网络错误清空旧坐标，过期 `/frame.jpg` 为 503，页面另有本地失效计时器；升级已刷新浏览器验证。
- 验证：Mac `130 passed, 89 subtests passed`，Pi `130 passed`；Pi `python3 -m tools.vision_health_check` 的 14 项软件注入检查通过，使用生成假帧和临时本机 HTTP 端口，不接触 USB 相机/串口。报告在 Pi `output/material_diagnostics/lifecycle_fault_injection_20261023.json`。覆盖空目标/恢复、检测异常/恢复、处理超时/迟到丢弃、采集阻塞失效、过期图像503、读帧失败/重开/恢复及唯一采集入口。
- 真实相机在线检查：Pi `output/material_diagnostics/lifecycle_live_7fQyF1cB/online_check.json`，30 秒 149 次查询、149 个新来源观测，采集/物料错误为 0，最大采集年龄 100.3 ms、结果年龄 241.2 ms；仅验证在线时效，不评价颜色精度/静止抖动。该目录 `shutdown_check.json` 记录实机正常运行时 SIGTERM 释放资源验证。最新完整源码服务日志为 `output/material_diagnostics/lifecycle_final_7jMT1hd7/service.log`，检查时 PID 47316，只有它持有 `/dev/video0`，诊断采样关闭、二维码/圆环/门控/串口保持关闭，显式 HSV、原 ROI/分辨率及基线 profile 不变。PID 是快照，后续重查。
- 回滚前源码备份在 Pi `backups/observation_lifecycle_20261023/before_lifecycle.tar` 与 `before_consumers.tar`。当前代码和此前接口改动仍未提交/推送。实体 USB 拔插、永久原生卡死的进程级恢复、长时间运行和实际串口主控未验；不把软件注入当成这些实测。

## 必要命令

Mac 登录（已配置别名，不要再写旧 IP/旧账号）：

```bash
ssh pi-vision
```

树莓派：软件测试，不打开真实相机或串口：

```bash
cd /home/cuiguo/competition_vision
python3 -m pytest -q tests --ignore=tests/test_lidar.py
v4l2-ctl --list-devices
```

树莓派：当前视觉启动命令。已有实例运行时先确认占用，在原服务终端按 Ctrl+C 停止，避免重复打开相机：

```bash
cd /home/cuiguo/competition_vision
python3 -m pi_vision.stream \
  --port 8090 \
  --camera /dev/v4l/by-id/usb-Generic_USB_Camera_200901010001-video-index0 \
  --width 1280 --height 720 \
  --materials --material-backend hsv --hsv-profile config/hsv_profile.example.yaml \
  --no-qr --no-rings --no-rotation-gate --no-undistort \
  --material-roi 480 60 520 450 \
  --preview-width 640 --preview-fps 5 --quality 55 \
  --capture-timeout 1 --result-timeout 1 --qr-timeout 10 --preview-timeout 5
```

Mac：检查本地隧道。仅在 8090 未监听时建立一个；已有隧道不要重复启动：

```bash
lsof -nP -iTCP:8090 -sTCP:LISTEN
ssh -fN -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:8090:127.0.0.1:8090 pi-vision
```

Mac 浏览器：`http://127.0.0.1:8090/`。状态接口是 `/state`，不是 `/api/state`。SSH 进程/网络中断后后台隧道不自动重建。

树莓派另一 SSH 终端：将绿/黑/浅蓝各一件完整放进 ROI，静止采样；只读现有服务，不再打开相机。其他分组更改 `--colors` 和输出名：

```bash
cd /home/cuiguo/competition_vision
python3 tools/material_acceptance.py \
  --url http://127.0.0.1:8090 --colors 4 5 6 --seconds 30 \
  --output output/material_diagnostics/green_black_lightblue.json
```

树莓派：加载 ROS 环境并检查：

```bash
source /opt/ros/jazzy/setup.bash
printenv ROS_DISTRO
python3 -c 'import rclpy; print("rclpy OK")'
```

当前 Ubuntu 直接用系统 Python 和 apt 依赖运行源码，以兼容 ROS；不要照搬 README 的虚拟环境/pip 安装覆盖此环境。`pyproject.toml` 的 OpenCV 声明为 >=4.8，而现机 apt 为 4.6.0；现有测试通过，但版本声明与环境尚未统一。

## 下一步与尚未完成

1. 当前直接推进任务框架及 ROS 2 收发，不继续安排非法码等人工逐项验收作为开发门槛。相机未固定，继续暂停像素精调；既有漏检/额外颜色与 USB 问题仍保留待后续真实安装条件复验，不因此宣称通过。
2. 真实单相机扫码/原料物料后端与局部对准软件流程已实现并部署，后者仅作独立软件/ROS 模拟验证。接下来为双相机角色、位置数字模板的印刷字体适配和真实电控适配；不依赖预览、不复制 HSV 检测器，不把未实现项填成占位成功。
3. 圆环/放置位置 1/2/3：按 PDF 平面布局的断续同心环及逐点旋转数字合成图已通过检测链测试；逐点方向自动候选，接口保留识别方向。`ring_no` 仍固定为 1，评分层号没有实现测量输出；真实打印字体、视角/遮挡、实物和相机平面标定未验，位置结果不得直接用于实机抓放。
4. 物料跨帧身份已在软件中实现保守短时跟踪，像素中心随当前帧更新，不平滑成运动目标；ID 会话内有效。还未以真实转盘运动/夹爪遮挡评估动态关联、漏检恢复和多同色物体交叉。
5. 完成相机/平面及抓取坐标标定，验证物料顶面高度影响与独立测量误差；检测框中心不是已标定抓取点。
6. 转盘停稳门控实机验收，覆盖进场即静止、换位、遮挡、丢失与恢复；现基线门控关闭。
7. 与主控明确任务控制、开始/停止/重扫、目标和确认接口；正式串口/显示屏/机械抓取闭环待验收。`app.py` 放置验证仍有成功占位，`fixed_distance_cm` 默认 20 是固定配置，均不是真实视觉测量。
8. 整轮扫码→抓取→放置→第二批码垛、长时间运行、故障恢复验收；ROS/预览接入后不得让浏览器断开阻塞识别。

## 使用边界

运行配置 `config/vision_config.yaml` 由样例恢复且被 Git 忽略；旧硬件标定、权重、数据未随私有仓库迁移。比赛要求已按上述三份附件核对摘要并标页码，但本文不是完整规则或最新通知；涉及争议、适用赛事或新增要求时重新核对原文。不要重新整理旧试错过程或追问已确认账号。雷达按用户本次明确请求开展设备/扫描基线测试，后续 ROS 接入与实车避障分别验收。

## 2026-11-01 参考视频方法第一轮实现

- 用户明确优先颜色、圆环放置和坐标输出，并授权大幅参考所给视频截图。新增可选面积质心
  `--material-center centroid`；HSV 阈值/形态分割保持原样，默认仍为 box。点定义贯穿
  Material、ROI、stream、ROS 和离线报告，旧框中心标定不适配质心。
- 新增 `MorphologyRingDetector` 和强制显式 ROI 的 ROS 入口选项：二值化→膨胀→均值→
  腐蚀→外轮廓质心，另做原始环线径向支持筛选；保留原 Hough 默认路径。与数字模板可组合，
  输出全图像素中心、代表半径和 center_method，不把径向支持峰数当评分层数。
- 新增 `tools/vision_method_compare.py`，对相同保存原图输出两种物料中心、两种圆环路径和
  六色/形态学阶段图；不打开相机/串口。历史诊断回放工具兼容旧 point_kind 缺省、区分质心
  并排除不可回放的会话 tracking 字段；仍核对颜色、坐标、分数和点定义。
- 离线证据：`output/reference_methods_20261101_material/` 对本地 `fixed_scene.jpg` 回放，
  红框中心 (759,202)、质心 (761,199)，颜色候选相同；左上黑色背景被错认黑物料、霍夫在
  黄色物料处给出无数字环候选，保留为误检事实，未因此改 HSV。没有抓取点真值，不能声称
  质心更准。`output/reference_methods_20261101_rings/` 为生成断续双环及数字图，两环
  1/3 中心 (140,150)/(490,150) 恢复成功；不是相机实物测试。
- 本轮只做可选算法、接口与离线对比，未完成转盘到达预测、排除法推断、同色底层验证或
  实际放置结果验证；未部署 Pi 正式源码、未启动相机/电控。相机固定与物料/地面/码垛高度
  标定依旧待完成。默认方法不切换，后续先以保存的错误样本与实物圆环原图比较两条路径。

- 本轮 Mac 全套视觉测试 `245 passed, 172 subtests passed`，包含新增 11 项：质心/框中心
  相同分割、ROI 全图坐标、断续环与数字、无效几何拒绝、ROS 点定义和旧诊断回放兼容。
  证据为软件单测、单张历史原图和生成图；未做 Pi 实机/性能/抓放验收。

### 用户照片单图回放（2026-11-01）

当前默认 HSV 直接处理用户提供的 1280×960 浅蓝物料原图，未改阈值、未设 ROI：浅蓝 ID 6
框中心 (658,480)、面积质心 (661,476)。另有黑色背景/阴影误检两个，质心分别为
(140,785)/(624,620)，不能当作真实黑色物料。报告、六色分割及标记图保存在 Git 忽略目录
`output/material_diagnostics/user_photo_20261101_163739/`。仅为单张图离线运行，无相机实时、
追踪、机械坐标或准确率验收；本例重点保留为黑色与阴影区分的回归样本。


### Pi 正式同步与手持物料实测（2026-11-01）

51 个视觉/ROS 相关文件已同步到 Pi 正式工程并逐文件 SHA256 核对一致，包含 d3c830c 新增方法；
Pi Git HEAD 仍为 f563961（采用文件同步，保留已有修改）。未覆盖运行配置、模型、图像。
原源码备份 `backups/vision_reference_20261101_1eu_fisu/before.tar`；Pi 软件测试
`244 passed, 1 skipped`（缺 Node.js 的网页 JS 模拟在 Mac 已通过）。

16:35/16:41 期间相机 USB 断开和枚举 -71，换口后用户重启 Pi。重启后设备恢复并可采集；
原 fe80 SSH 路径一度超时，改用本次核实可用的 HostName=cuiguo.local 临时覆盖，保留既有
密钥与主机身份验证，不修改 SSH 配置、固定 IP 或网络设置。Mac 8090 IPv4 隧道已恢复。

唯一采集服务检查时 PID 1494，显式 HSV、1280×720、原 ROI (480,60,520,450)、框中心输出、
关闭扫码/圆环/转盘门控/去畸变/串口。服务持续运行，预览 http://127.0.0.1:8090/；
原图诊断保存达到 30 张后停止写图，不停止采集。运行命令和证据在 Pi
`output/material_diagnostics/material_live_test_20261101_164719/`，Mac 摘要/标记图在
`output/material_diagnostics/pi_live_20261101/`，均 Git 忽略。

20 秒 HTTP 抽样有 99 个不同来源的新鲜观测，浅蓝 ID6 检出 99/99，无请求错误/采集错误；
框中心 x576—630、y329—374。用户明确手在移动，这段是手持移动场景，不能作算法静止抖动
或固定安装精度证据。保存的30张原图离线同帧比较，框中心/质心均检出浅蓝30/30，10张出现
黑色背景误检；sample_0030 框中心(602,338)、面积质心(600,339)，不是当前持续锁定坐标。
最后一次查询为新鲜 no_target、目标空，旧坐标未沿用；不是人为移出/恢复的完整验收。
当前实时服务仍用框中心，质心由同一采集入口保存的原图离线计算，未启动第二个相机实例。
没有实际机械/放置验证，也不能由单件浅蓝结果推出六色正确率或长时间 USB 稳定。


### 中央单目标规则与固定场景待测（2026-11-01）

用户明确相机已固定，算法只看中间颜色，外围可能偶然拍到其他物料。已新增可选
`--material-target-window 480 240 320 240`（原图1280×720，中心640,360）；先按空间
选中央最近候选，再由下游按请求颜色匹配，窗口空或距离近似歧义时不输出目标。
HSV 分割保持不变，stream 状态/诊断和回放保留窗口。stream 也新增 --material-center
可选 box/centroid。规则写入 AGENTS.md，不得将外围颜色回退为中央目标。

相关代码已备份在 Pi `backups/center_window_20261101_w1msim63/before.tar` 后同步；
Mac全套249 passed、172 subtests passed，Pi248 passed/1 skipped。此前失效的
stream PID1494已SIGTERM退出；当前没有相机采集服务。原因是USB相机16:50再次断开、
枚举-71；用户表示处理好了后复查仍没有/dev/v4l/by-id或新的成功枚举记录。不得沿用
上轮(602,338)/(600,339)坐标为本次固定测试结果。USB设备恢复后再以唯一入口启动，
原HSV/ROI保持，附加中央窗口选项；届时重新限量保存原图并测静止坐标。


### 更换端口后固定场景完成（2026-11-01 16:54—16:55 快照）

用户换到另一 mtconn 端口后，相机在 usb2-2 成功枚举，固定设备路径恢复，无占用。
已启动唯一 stream，检查时PID2679，完整命令在 Pi
`output/material_diagnostics/center_fixed_20261101_165441/command.json`。显式HSV、1280×720、
原ROI(480,60,520,450)、中央目标窗口(480,240,320,240)、实时面积质心，扫码/圆环/门控/
去畸变/串口关闭。服务继续运行；限量60张原图后停止写盘，检测和预览继续。

固定后20秒HTTP抽样99个不同来源新鲜观测：99/99只输出中央浅蓝ID6，外围颜色输出0，
质心x642—643/y345—345，采集/处理/请求错误均0。此为单件固定场景的短时间重复性证据，
不是六色泛化、绝对定位/抓取准确率或长时间USB稳定性保证。99观测非全部推理帧。
当前质心来自整块可见颜色轮廓，包含侧面，不是单独顶面圆心或机械抓取点。
标记图sample_0049质心(642,345)，绿色框为中央目标窗口，旁边黄/蓝物料未被输出。
Pi证据：同目录summary.json、states.json、raw/及annotated.png；Mac副本
`output/material_diagnostics/center_target_20261101/`，均不进Git。后续保留中央选择，
中央没有明确物料时不回退外围；使用夹爪前须验证所需顶面点定义并标定。


### 中央物料再次抽样（2026-11-01 16:58）

用户要求再次测试，复用现有唯一采集服务、原HSV/ROI/中央窗口，不改阈值或追着物料改窗口。
15秒58个不同来源新鲜观测均输出浅蓝ID6，质心x580恒定、y245—246，无空帧/请求错误。
起始预查质心(580,247)，位于中央窗口上边缘附近；统计窗口末尾(580,245)。
采集错误累计37、重开3次为此前历史，本轮错误计数37→37未增加，不能称整个服务无错误。
Mac证据 `output/material_diagnostics/center_retest_20261101_165841/`；标记图使用HTTP预览
叠加临近状态，仅示意中央窗口，原图坐标以JSON统计为准。没有更换相机入口或进行机械动作。

### 相机质心偏移复核（2026-11-01）

用户指出单图图1标点正常、相机图2标点明显偏下，期望以图1效果检查相机链路。
回放图2同一份640×360未标注JPEG预览，按缩小后的ROI和中心窗口计算，面积质心为
(290,123)，对应原图约(580,246)，与当时实时结果(580,245)接近。黄色实际分割轮廓
包含顶面和侧面，复现偏下原因；这不是已证明的实时坐标缩放错误，也不排除其他场景问题。
此前HTTP预览与状态分开读取的标记图并非严格同帧，后续坐标验证使用同源原图/结果。
大绿色框是候选中心选择窗口，不是物料框，不用于裁切顶面；此前图1小绿色框却表示框中心，
标记含义不统一。已输出去掉选择窗、只画颜色轮廓与面积质心的核对图：
`output/material_diagnostics/center_marker_review_20261101/camera_contour_review.png`。
本轮未更改颜色阈值或生产中心算法。整块颜色质心不能当单独顶面圆心或机械抓取点。


### 顶面定位已部署并实测（2026-11-01 17:53—17:54 快照）

用户明确相机拟安装在物料上方、用于电控机械抓取，要求单独顶面定位，不沿用整块颜色质心。
新增 pi_vision/top_face.py；对圆顶可见边界、椭圆拟合/残差、颜色重合/原始边缘覆盖和歧义
作检查，并结合顶面与侧面亮度对比避免选到较大侧面轮廓。--material-center top_face
贯穿stream/ROS，point_kind=top_face_ellipse_center，不成功时不回退整块质心；先按整块
物料锚点选中央目标，再验证顶面，不能中央失败后挑选旁边物料。box/centroid保留可选。
HSV分色阈值未变；新候选不改业务色号。内部无效候选不对HTTP/ROS/串口发布定位点。

Pi改前源码备份 `backups/top_face_20261101_cb__d9al/before.tar`。最终服务检查时PID4650，
唯一占用/dev/video0。参数/证据目录
`output/material_diagnostics/top_face_final_20261101_175339/`，command.json为完整启动命令。
显式HSV、1280×720、原ROI(480,60,520,450)、中央窗口(480,240,320,240)、top_face实时模式，
不启二维码/圆环/门控/去畸变/串口/动作端；同一采集入口保存最多60张原图，服务继续运行。
早期试验有顶面/外轮廓切换，已修正候选亮度比较后重新启动并重新抽样；旧目录不作为最终证据。

最终20秒97个不同来源新鲜观测：97/97为中央浅蓝ID6顶面，x649—649/y243—243，
采集错误0→0、请求错误0，抽样中最大处理时长90.2ms。原图sample_0030(source175)
独立同帧回放顶面(649,243)，旧面积质心(648,250)，顶面轴长约192.7×212.2px，
几何与实时记录核对一致（matches_recorded=true），不是预览与临近状态拼接的标记。
Mac图/报告在 `output/material_diagnostics/top_face_final_20261101/`，均Git忽略。

Mac完整视觉测试256 passed/172 subtests，Pi255 passed/1 skipped（Node.js网页脚本在Mac通过）。
含新增顶面/侧面分离、俯视、ROI原图坐标、小亮斑/无圆顶/裁断拒绝、中央顶面失败不选旁边、
近圆形侧面外轮廓抑制测试。仅实测本场景浅蓝圆顶、像素重复性；不代表六色/异形顶面/
遮挡反光泛化、绝对圆心精度、长时间USB或实际抓放精度。相机固定/光轴近垂直顶面是安装意图，
仍须按本车姿态、高度、畸变和机械轴方向标定，旧质心/框中心标定不复用。


### 用户位置2圆环截图回放（2026-11-01）

用户提供623×737单圆环截图，使用现有默认MorphologyRingDetector和RingDetector各自接
NumberedRingDetector(orientation=0)，未改参数/阈值，均恢复圆心(321,394)、position_id=2。
形态学中心来自二值化→膨胀→均值/重二值化→腐蚀后的外轮廓质心，并检查原图径向环线支持；
不是数字2质心、YOLO框中心或已标定毫米点。数字模板分数0.92325、类别间隔0.17686，
为形状相似度而非识别概率。ring_no仍是旧兼容字段1，不能当放置位置或实际评分层号。
证据 `output/material_diagnostics/ring_user_20261101/` 含source.png、report.json、阶段图和
两方法标记图，Git忽略。本轮不修改生产算法、不启用Pi圆环或停止现有物料相机服务。

用户再次核对取放流程：设计是按任务码识色选目标→顶面对准/停稳确认→抓取；放置按位置码
识别对应1/2/3环、对准圆心再放。排除法第三色仍未实现，推断不能替代目标当前坐标；
第二批暂存仍需同色底层验证，不能当空环放置。圆环算法已有但尚未接真实对准/放置动作闭环。


### 真实相机圆环ROS发布计时（2026-11-01 18:38—18:39 快照）

用户要求记识别判断到发坐标的时间。已部署VisionBackend同机单调时间分段、create_node可选
发布事件记录，以及tools/ring_camera_timing单采集入口+8090预览+隔离ROS本机订阅计时工具。
备份Pi `backups/ring_timing_20261101_h_90v4c9/before.tar`；物料服务PID4650已停止释放相机，
计时服务当前检查PID5936，唯一相机，domain74/LOCALHOST，无串口/动作/任务主控。
首个启动因rclpy初始化顺序错误未产生有效测量，已修正；有效证据只用evidence_retry1。
完整命令在 `output/material_diagnostics/ring_timing_20261101_183755/command_retry1.json`，
有效日志/数据在该目录 `retry1.log`、`evidence_retry1/events.jsonl`。

当前测位置2、形态学圆环+数字、显式HSV但仅开启rings，相机1280×720，ring ROI(160,0,960,720)。
有界300秒测试，到期/终止自动写最终报告并释放相机，不常驻。用户应让圆环完整进入ROI。
采样快照166次有效坐标的中位数：读帧完成→结果登记54.381ms；结果登记→ROS发布调用返回
2.634ms（P95 3.363ms、最大6.114ms）；结果登记→同机测试订阅者收到3.389ms；
读帧完成→同机收到57.531ms（P95 67.563ms、最大67.802ms）。首次确认坐标在请求后
595.541ms收到，包含3新帧/0.2秒连续确认等待。位置2坐标随相机/目标移动变化，非静止精度验收。
这不等于串口发送、单片机接收或传感器曝光到结果，166是坐标输出数不是识别正确率。
Mac快照 `output/material_diagnostics/ring_timing_setup_20261101/interim_summary.json`。
Mac全套258 passed/172 subtests，Pi本轮相关54项通过；计时功能未接真实机械闭环。


### 相机圆环标记图（2026-11-01 18:45 快照）

前次300秒ROS计时已正常结束，最终392次有效坐标输出；识别完成到ROS发布返回中位数
2.979ms，读帧完成到本机接收中位数71.023ms，不含串口/单片机。新拍照前核实相机无占用，
按同一默认形态学+数字算法及ROI(160,0,960,720)处理新拍原图，未改阈值。
18:45:44.556的1280×720原图识别位置2，圆心(737,340)，数字相似度0.878、间隔0.160。
同帧红十字为圆环外轮廓质心，青色圆为代表半径示意（不是最外环或评分层边界）。
场景为相机拍摄电脑屏幕上的圆环图，不是打印实物/真实地面验收；单张候选不代替连续确认。
Mac/Pi均留存output/material_diagnostics/ring_camera_picture_20261101_184542/中的
raw.png、annotated.png、report.json及形态学阶段图，均Git忽略；结束已释放相机，无机械动作。


## 雷达静态底图、实时扫描与路线预览（2026-11-01 20:03 快照）

- 用户要求固定比赛底图，同时持续实时扫描，用参考截图的25格、9点12边和A*方式观察
  障碍物与绕行；不要求现在量初始位置。用户最新明确：雷达尚未放到比赛场地上。
  当前真实回波只代表雷达当前位置周围环境，不能算比赛场地障碍物验收。
- 名义底图已按全国命题T第3—4页建立：2400方场、400十字通道、450黄区、300启停区。
  配置config/field_map_national_2027.json精准放行Git忽略例外；其它设备配置仍忽略。
  tools/build_field_map.py生成SVG/JSON/预览到output/field_map_20261101（忽略）。
  原料装置/二维码实际位姿未确认；地面颜色不是雷达墙，不伪造SLAM地图。
- tools/lidar_sdk_capture.cpp复用已有官方SDK、只用明确by-id；tools/lidar_view.py/html
  和lidar_navigation.py集成静态场地图、持续相对雷达点云、相邻回波候选、25格滚动证据、
  9节点12边A*。格界0/.55/1/1.4/1.85/2.4m，右下0蛇形至左上24；路网x/y为
  .35/1.2/2.05m工程选点，不是官方站点。半径加余量默认.18m仅假设，静态禁行区和
  边界参与边阻断。至少3个不同扫描命中、最多14圈、证据1秒到期变未知，不标成free。
- 页面同一入口http://127.0.0.1:8766/，已有Mac隧道PID23480复用。Pi独立部署目录
  /home/cuiguo/ydlidar_work/live_view_20261101/，未覆盖视觉项目、未操作底盘/相机。
  19:59:05启动服务PID14692、最长3600秒，约20:59自动结束；状态只是快照。
  运行证据该目录runtime_20261101_195905，前300圈留存、SDK日志约2MB上限、退出写summary。
- UI默认pose未知，右侧一直可看真实扫描。可选estimated_start叠加仅是假设流程预览，
  manual_stationary仅适用于确实测过的静止位置；无自动定位或移动位姿跟踪。
  用户澄清尚未上场后，已POST pose=null，回读occupied为空、route为static_nominal_preview。
  不把之前估计叠加的占据格当现场障碍位置；所有路径valid_for_motion=false，无运动指令。
- 首次SDK基线5038圈有序退出，启动阶段6次checksum错误，后续约8分钟未增长；这些
  历史数据不代表障碍正确率。新服务快照约10Hz/数百点，尚未核对实物真值或坐标朝向。
- 软件完整范围273 passed/172 subtests；导航/时效相关15项通过。真实浏览器桌面与390px
  手机检查无横溢出，断网时点数0、候选/动态占据/实时路线清空，恢复后接新圈。
  output/lidar_review/integrated-*.png留证（忽略）。软件测试不替代比赛障碍物检出率、
  自动初始定位、距离精度、车体运动碰撞、安全停机与整车避障验收。
- 下一步：用户把雷达放到场地后，先观察障碍物移动与真实回波关系、核对传感器方向；
  再完善启停区随机摆放的初始位姿与移动定位接口，不将反复手工量位置当开发门槛。
  运行/编译和接口说明见LIDAR_VIEW.md，启动命令必须带--field-map。

### 断电重启恢复（2026-11-01 20:08 快照）

用户给树莓派断电后要求恢复雷达。20:06:31 SSH 重新上线，确认 by-id 串口存在且无占用；
启动独立 lidar_view 服务 PID1171，目录 `/home/cuiguo/ydlidar_work/live_view_20261101/`，
新运行输出为同目录 `runtime_20261101_200817`，最长3600秒，命令仍带 `--field-map`。
Mac 原8766隧道在断电后已失效，已用 `ssh -fN` 重新建立，页面入口不变。
20:08 回读 `/state`：valid=true、约9.8Hz、207点/圈、pose=null；仍未把当前环境当比赛场地障碍。

### 再次启动命令（2026-11-01 20:15 快照）

用户要求再次开启并希望以后自行启动。上次后台进程已退出；本次改用 `nohup setsid` 脱离SSH会话，
启动Pi PID1288，运行目录 `runtime_20261101_201500`，Mac隧道重建PID36357。回读状态
valid=true、约10.0Hz、361点/圈、pose=null；串口仅由PID1288占用。以后需先执行状态检查，
确认没有已有 lidar_view，再执行LIDAR_VIEW.md对应命令；不要重复启动以免串口冲突。

### 现场启停区1与网页显示修正（2026-11-01 20:37 快照）

用户将雷达放到实际启停区1，并提供现场斜视照片。照片中的地布、桌腿、墙、箱子和人员/设备
会产生真实激光回波；雷达不能读取地布印刷线、颜色或比赛区域语义，因此现场点云不会像相机照片
或静态规则图。网页底图是左下原点的二维俯视名义图，不能直接与斜视照片外观一一重合；启停区内
的具体雷达坐标和传感器朝向仍未测量，不能把回波精确投到场地格子。
- 页面改为默认1.5m近场显示，候选圈默认关闭，增加“与现场照片同向（旋转底图180°）”选项；
  继续保留右侧雷达相对坐标，避免场外远处物体把画面塞满。
- 新增 `tools/lidar_display.js` 显示缓冲：短暂网络抖动保留上一圈灰色回波最多5秒，灰色数据不参与
  占据/路线；恢复后自动接收新圈，超时才清空，不要求手动刷新。服务器为每次运行生成session_id，
  防止断电重启后的旧帧混入新会话。
- 更新后的py/html/js已部署到Pi `/home/cuiguo/ydlidar_work/live_view_20261101/`，当前服务
  PID10548、运行目录 `runtime_20261101_203429`；部署后回读约10Hz/377点，pose先继承了测试用
  的估计值，随后已POST `{"pose":null}` 清除，当前恢复为pose_unknown。
  浏览器验证断网2.2秒时仍显示灰色旧图，5秒后清空，恢复后自动回到实时扫描；无横向溢出。
- 相关导航与服务测试15项通过，`node --check tools/lidar_display.js`通过。下一步是现场核对雷达相对
  启停区的朝向/位置，再决定是否使用manual_stationary叠加；不把估计叠加当定位结果。


### 场地照片配对与位姿显示校准（2026-11-01 21:10 快照）

用户确认雷达已摆入启停区1，提供4组现场/网页对应照片，指出远端障碍显示偏移且怀疑方向错。
复核静态地布：启停区2在左上、启停区1在左下、粗加工区在上、暂存区在右；与“照片同向”
180°显示一致，底图自身方向正确。错误来自服务残留测试位姿[2.25,2.25,180]，这只是启停区
中心估值，朝向也未经核实；本轮已清为null，回读pose_unknown，不能继续将旧红格当真值。
- 网页新增底图点击选雷达坐标、传感器+X方向0—315°每45°、分别应用估计/已测静止位姿；
  未设置位置/朝向时不投影扫描。地图变换已提取到 `tools/lidar_display.js` 并增加往返/极向测试。
- 雷达相对坐标默认半径3.4m，覆盖2.4m场地角到角的理论距离；仍可选6m查看场外回波，场地投影裁剪到2.4m。
  远处扫描点不会因此丢失，但超出场界的点不会误画到场内。
- 已部署Pi独立目录，浏览器桌面/390px检查无横向溢出，雷达约10Hz/363点、位姿null。
  `node tests/test_lidar_display.js`通过，导航/服务15项通过，git diff --check通过。
- 图片只能说明平面图的朝向与场景布局，不能精确给出雷达旋转中心或T-mini坐标轴方向；不能从照片
  自动推断准确yaw。用户可在网页地图点击雷达中心，选择真实朝向后查看重合；场地2.4m尺寸不限制雷达量程，
  但近地扫描高度、遮挡、目标材质仍会影响实物检出。无自动定位或移动跟踪。

### 安装照片与实时状态复核（2026-11-01 21:22 快照）

- 用户补充雷达平放安装近照和场地/障碍物照片。近照显示雷达扫描盘朝上、安装面看起来水平；没有可辨认的
  传感器零度标记，不能用线缆出线方向当作已校准的朝向。水平与否仍需现场尺/水平仪确认。
- 21:22—21:23 Mac 隧道仍通，Pi `/state` 持续为 `live/valid`、约 9.9—10.0 Hz、每圈约 301—389 点。运行时位姿为
  `[2.20, 2.20, 135°]`、来源 `estimated_start`，这是图片/近场回波支持的临时叠加估值，不是测量、定位或标定。
  连续回读中占据格曾从 3、4 变为 4、12、16、17，阻断边及路线也变化（`no_path`/路径预览）；始终
  `valid_for_motion=false`。票数/格子/路线会随实时扫描窗口变化，不应直接认定为照片里的黑色圆柱；
  真实扫描已在线不代表场地坐标准确。
- 当前有两个本地页面入口：`8766` 为雷达实时页，`8767` 为静态底图页。若地图上没有动态标记，先确认打开
  `8766`；右侧相对坐标点云用于检查传感器实际回波，场地图叠加仍受上述临时位姿影响。

### 恢复照片前的简洁集成页（2026-11-01 21:32 快照）

用户选择恢复照片前版本：1.5m近场、候选圈关闭、底图不叠位姿。只回退网页交互，不停止雷达进程；
恢复“场地叠加/启停区1估计/取消叠加”简洁控件，移除地图点选和yaw选择，默认范围恢复1.5m。
保留照片同向开关、5秒灰色观看缓存、网络自动恢复和A*预览。当前Pi服务仍PID10548，不重启，
只更新lidar_view.html；POST pose=null后回读unknown、occupied=[]，雷达约10Hz/378点。
网页已重新加载，显示简洁集成页。docs/LIDAR_VIEW正文同步此现状；估计按钮仍是[2.25,2.25,180]
的演示假设，不应用即不投影，现场实测前保持未知位姿。

### 8766无法访问恢复（2026-11-01 21:36 快照）

Mac端SSH隧道PID36357仍监听，但Pi在线、雷达by-id存在、串口空闲且Pi8766无监听；上一服务
PID10548的一小时bounded runtime已自然结束（约21:34），因此Mac收到connection reset。未重复启动
已有采集。21:36启动新服务PID54552，输出目录 `runtime_20261101_213611`，仍限3600秒；
隧道复用。验证 `/state` live/valid、10.0Hz、371点、pose=null，HTML返回1.5m集成页；
浏览器原“无法访问”标签已重载并显示实时扫描。约22:36服务会再到期。


### 雷达同步校准与远端显示（2026-11-01 21:58后快照）

- 用户明确要求问答同步、多次换格验证；已配合14→13→7→10→20、20移出及自选12；用户确认12显示正确后明确“这轮先停”，停止人工联调，雷达服务继续。
  初始`pose=null`导致底图无动态投影；恢复临时旋转后14可显示，但13被旧变换投到场外。
- 原始差分证据：14处小目标在40/40圈存在，移走后0/40；放回约(.183,.842)m，移到13后
  约(.581,.817)m。补充可配置`angle_sign`并在导航与网页同时应用，SDK/原始扫描不变。
- 当前静止预览配置为`pose=[2.15,2.20,-93.52]`、`angle_sign=-1`、`pose_source=estimated_start`，
  是格子级现场估计，未测旋转中心真值或毫米误差；默认angle_sign仍+1。车/雷达移动后须重新核对。
  用户明确确认13、7、10、20、12显示；同组参数未按7/10/20/12分别再调。
- 实时抽样：13小目标30/30圈检出，证据累计后28次读取占据13；7占据40/40；
  10目标距离约2.08m、50圈31圈直接回波，50/50读取占据10；20约2.65m、60圈22圈直接回波，
  原1秒窗口53/60读取占据20，用户确认偶尔消失；最后用户自选12约1.36m，40/40圈直接回波且40/40读取占据12，用户确认显示正确。仅为本次固定场景，不代表所有距离/位置检出率。
- 本次采用`evidence_ttl_s=2.0`累计稀疏回波，最多32圈、至少3个不同扫描命中。默认仍1秒；
  所有失效扫描/断连仍不参与占据。部署后70/70读取占据20；用户拿走后45/45读取无20，只有4。
  无移走瞬间时间戳，不声称实测清空延迟；窗口机制最多保留约2秒已知命中。绿色点仍为当前圈。
  4号格包含雷达/车体附近真实回波，未测车体轮廓，未直接当作本车屏蔽区或黑色圆柱。
- 修复`lidar_view.html`回退后的残留px/py/yaw DOM访问，避免非空位姿初次加载时轮询报错。
  右侧观察半径仍1.5m，不影响左侧全场投影；远处单点虽不满足两点候选圈，仍可参与占据。
- 通过`--preview-settings`显式载入Pi独立目录`stationary_preview.json`；网页从
  `/preview-settings`读取，“恢复本次静止校准”恢复同组配置，避免演示按钮覆盖本次结果。
  文件保留在独立设备目录与Mac忽略证据目录，不作为通用硬件标定提交。启动参数见LIDAR_VIEW.md。
- 当前Pi服务PID70399，目录`/home/cuiguo/ydlidar_work/live_view_20261101/runtime_sparse_20261101_215736`，
  最长3600秒，约22:57自然到期；Mac8766隧道复用。更新前核对命令与SDK子进程，有序停止后确认串口空闲，
  未操作相机/底盘/电控。旧版本在同目录`before_angle_calibration/`、`before_sparse_window/`。
- 导航/服务20项测试及display JS通过，含14→13回归、反向配置改变清旧证据、稀疏回波累计及移走/失效清空；
  Playwright实页确认正常加载、2秒窗口及恢复按钮。证据`output/lidar_calibration/`（忽略）。
  所有路线仍`valid_for_motion=false`，无自动定位/移动跟踪。本轮已按用户要求结束人工问答测试；保持本配置与服务运行。下一步在安装位置改变或底盘运动前重新核对，不将静止格子验证当作移动定位或自动避障验收。


### 双启停区预览配置（2026-11-01 22:08后快照）

- 用户将雷达/车从区1移到区2，明确朝向不变，澄清障碍物仍在12。原区1位姿导致整体错投；
  保留yaw=-93.52°、angle_sign=-1，仅将位置估值切为[2.15,0.20]后40/40圈目标回波落在12，
  距离约1.33m，用户确认“12恢复显示了”。区2仅核对这一处，未作全格精度验收。
- 网页新增“起始启停区”1/2和“应用启停区配置”。POST `{"start_zone":1|2}`由服务从已验证配置取参数，
  一并换位姿及路线起点（1区8、2区2）并清空旧回波累计；不重建底图或改扫描SDK。
  选择但未应用不改变运行参数。直接手动改pose或清空pose会清除start_zone标签，避免误报当前区。
- Pi独立目录stationary_preview.json增加start_profiles字典，两区独立保存pose、yaw、角度方向、累计时间等；
  本次启动区2。运行切换不自动改写文件，重启后仍默认2，赛前需核对当前区。兼容旧单组文件；
  未配置的区拒绝切换，不捏造定位。两区本次都沿用同一朝向，不对比赛车头朝向规则作推断。
- 更新后PID78614、目录`/home/cuiguo/ydlidar_work/live_view_20261101/runtime_two_starts_20261101_220803`，
  最长3600秒约23:08到期；已有隧道复用。更新前核对70399/70401命令并有序结束，确认串口空闲后重启；
  旧文件和单区配置备份在before_two_starts/，不覆盖视觉或操作电控。
- 22项导航/服务测试及display JS通过，含双区切换/起点同步/清旧票/拒绝未配置区；部署后30/30读取占据12。
  Playwright验证下拉仅选择不应用、应用区2后路网起点与状态一致；1365/390 CSS宽度检查无横溢出。
- 该功能仅为选区初始化的静止预览。启停区内随机偏放/转向的自动初始定位、行驶里程计/IMU位姿跟踪尚未接入。
  不能把这两组固定pose当作自动定位；比赛不必现场改源码，但复现初始基准或补定位仍是后续必做项。
  所有路线继续valid_for_motion=false。证据output/lidar_calibration/start2_*.json及two_start_preview.json。


### 起始区由外部信号指定（2026-11-01 最新决定）

- 用户要求比赛运行时不通过电脑选区；随后明确该问题已解决，到时由对方发送信号提供起始区。
  当前采用外部控制端提供1/2区号、树莓派初始化对应配置的方案；不继续推进相机或纯雷达自动判区。
  发送方具体设备、通信链路、消息格式、确认/时效及启动顺序尚未确定，不假定已有串口或ROS协议。
- 已有HTTP预览接口能用start_zone=1/2选区，双配置和底图可复用；外部设备信号到树莓派的适配尚未接入或实测，
  不把方案确定等同于通信已联通。正式任务应收到并确认有效区号后再初始化，不以调试默认区2充当实际开局信息。
- 两区共用地图/格号/规划算法；区号用于选初始位姿和起点。区号信号不提供区内精确位置、朝向或移动定位，
  仍需约定初始摆放基准并后续接入位姿跟踪。已有静止预览不代表正式整车导航已验收。


### 八段规划、车体回波与演示动画（2026-11-01 22:37后快照）

- 用户明确车已回启停区1、障碍在13；要求当前启停格不标红、参考图8段一次算好，并解决20/24远端漏检。
  新增lidar_mission.py：启停→扫码→原料→加工→暂存→原料→加工→暂存→启停，POST action=plan_mission
  以同一实时证据快照计算，state.mission_plan保存8段/总长度/来源seq与配置；轮询不重算。
  新回波阻断旧路线提示重算，配置变化清计划，失效数据不作实时规划。所有路线仍valid_for_motion=false。
- 名义接近点qr5/material7/processing1/storage3，home区1为8、区2为2；并非实测机械停车点。
  配置mission_stations可替换；不得把截图18.27m当目标。初次实机快照#128（本次会话）占据1/13、阻断4/8，
  8段均可通，总长17.00m，避开13附近边8；JSON在output/lidar_calibration/eight_routes_plan.json。
  后续人/设备产生新阻断时该旧快照被标为需重算，动画仍只演示保存的几何。
- 当前启停格区1=4/区2=0不作红色填充；仅该格内r≤0.30m回波从导航证据剔除（配置self_mask_radius_m），
  整格不是free，遮罩外真实回波仍可阻断边。0.30为预览假设，非经测量的本车轮廓或实车安全包络。
- 原采集器固定10Hz改为可选6/8/10，重新链接现有厂商SDK（原库/校验未改）。本次6Hz实机首55圈，
  每圈点数中位599、占据13为40/55、其它格有周围回波；不声称13恒定检出。
  配置近场2秒/≥1.8m远端4秒、最多64圈/至少3不同扫描命中；20/24显示实际命中统计，无回波仍未知。
  默认观察半径改3.4m，不伪造缺失远端点。用户在20实测请求时选“先看8段路线，暂不移动”，所以20/24
  在6Hz+4秒新配置下尚未完成实测。下一步用户愿意移动时再做，不把参数改进当实测完成。
- 用户随后只要简单动画：网页已加入播放/暂停/重播与1/2/4倍速，白色箭头按8段顺序移动。
  RoutePlayback在JS单独实现，默认2倍速全程约11秒，站点停0.6秒（展示时间）不代表真实抓放动作。
  JS测试验证位置、暂停、8段顺序、重播/结束；实页实际播放至“8段演示完成”，暂停/重播通过。
- 导航/服务/批次28项测试及display JS通过，新增tests/test_lidar_mission.py，含两区8段连通、避阻断边、
  快照不改写/新障碍警告、缺数据拒算、稀疏远端过期、自身遮罩不屏蔽整格真实障碍。网页1365/390宽度无横溢出。
- 更新前核对78614服务与SDK子进程后有序结束，确认by-id空闲后启动PID100728，目录
  /home/cuiguo/ydlidar_work/live_view_20261101/runtime_eight_routes_20261101_223705，最长3600秒约23:37到期。
  同目录before_eight_routes保存旧源码/二进制/配置；stationary_preview.json现在启动区1、两区均保存0.30遮罩/4秒远端窗口。
  新采集器启动需--scan-frequency 6；新增lidar_mission.py必须与其它文件同目录部署。
  Mac8766隧道复用，未动相机/底盘/电控。后续外部区号通信适配、初始精确位姿和运动定位仍未接入。


### 双启停区静止试校准与边界显示（2026-11-02 17:37后快照）

- 当前联网快照：旧IPv6 SSH超时、mDNS失败；用户提供IPv4 `172.16.24.76`，通过`ssh -o HostName=172.16.24.76 pi-vision`使用既有密钥及主机身份校验连接。未修改永久SSH配置或公共网络；地址不能当常量。Mac8766隧道已恢复。
- 用户配合区1、区2分别摆车且每区内不动，区2车头保持与区1在场地中平行。无尺，目测格中央仅作格号验证；拟合改用地布印刷黄区角点、手工对准圆柱底面中心。用名义直径50mm，把目标表面中位点沿射线补25mm，拟合二维刚体变换，不改场地或测距比例。直径、实地尺寸、精确摆放未实测。
- 区1角点世界坐标(1.4,1.4)、(0.55,0.55)、(1.85,0.55)m，各55圈有目标回波。试校准pose=`[2.071763,2.201801,-91.322914]`；角点拟合残差35.9/37.6/56.4mm。区2角点(1.4,1.0)、(0.55,1.85)、(1.85,1.85)m，各55圈有目标回波；pose=`[2.081762,0.192084,-90.876521]`，残差33.1/35.0/57.9mm。两区angle_sign=-1、pose_source=estimated_start，残差包含手工摆放/模型/测距影响，不是已验定位精度；表面圆模型交叉计算另有约2cm差异。
- 配置保存Pi独立目录`stationary_preview.json`的两组start_profiles，顶层当前启动区2；区1结果保留。两区仍共用地图，区号由未来外部信号提供，通信适配未接入。区内随机偏放/朝向变化及运动定位未实现，当前仅对这次固定摆放成立，不能封装为任意摆车可用的精确定位。
- 区1独立验证：24直接回波55/55、占据55/55；13偏左曾55/55占据13和6，6点只跨过x=1.4边界0..21mm，与13点连成小片；拿走后55/55两格均清空。20偏19侧55/55读取仅20、9靠0侧55/55读取仅9。早期未修正位姿的20中央直接命中41/55、累计55/55，不能把累计读数当直接检出率。
- 区2独立验证：24直接回波55/55、累计占据55/55；12、20各55/55读取占据对应格。最后移出55圈中12/20/24均无占据；1号格仍有3/55次短暂占据，来源未确认，未人为屏蔽。无移出瞬间时间戳，不报告实测清空延迟。
- 显示改动：新增30mm格边显示带，仅有贴边累计证据或内部命中不足3圈时橙色“贴边待确认”；持续内部回波红色。字段boundary_only_cells/interior_evidence_cells。此带不是误差上界，不判物体数量，不把橙色判可通行；原始点、occupied_cells、blocked_edges、A*证据全部保留。真实13偏左回放与实页为13红/6橙/阻断边8，用户确认“现在很好”。所有路线valid_for_motion=false。
- 用户要求补二维码区：按field_map中qr_board距顶1100..1300mm范围绘制原图右边界短线及“二维码区/板位置范围”；180°照片同向为左侧中间。未假定精确板位姿或改名义接近节点。HTML热更新后须刷新一次；实页1365/390px检查，修正双行小字重叠后确认，无横溢出。控制台仅favicon404。
- 验证：30项导航/服务/批次测试及display JS通过；新增边界不丢阻断、内部真实回波仍红、远端过期和断连清空回归。现场记录是静止诊断，未操作相机/电控/底盘，未验整车运动、绝对厘米精度或比赛稳定性。
- 当前服务快照PID27131，6Hz，目录`/home/cuiguo/ydlidar_work/live_view_20261101/runtime_zone2_fit_20261102_173715`，最长3600秒约18:37到期，旧配置在该目录settings_before.json。此前部署均核对精确服务命令、SIGTERM退出并确认雷达by-id空闲后重启；代码备份在runtime_boundary_20261102_172123/before/。启动阶段SDK曾报校验错误，随后live收帧，未更改厂商SDK。
- 证据及分析：本地`output/lidar_calibration/20261102/`的原始JSON、corner_fit.json、zone2_corner_fit.json、final_stationary_preview.json、分析脚本及boundary/qr截图（均忽略）。Pi本轮逐点数据由record.py写入runtime_calibration_20261102_165931。设备配置和现场数据不提交Git。
- 下一步：用户确认结束则保留当前配置；若安装/摆放变动需重新核对。后续确定可复现初始基准，接入外部区号与移动位姿，之后再谈真实路径执行。本轮雷达相关代码与上下文已提交并推送origin/main，提交eda1885；现场数据/设备配置不上传，其他视觉工作区改动单独保留。

- 用户随后要求实际计算8段：在线区2、同一实时快照seq3083生成计划39ed59983b9a428b8618e902de31fb32，8/8可通，总长17.00m；快照占据11、阻断边3（来源未确认，不认定为比赛圆柱）。各段长度0.85/1.70/3.40/1.70/1.70/3.40/1.70/2.55m，网页已保存快照并显示。名义接近点、静止估计位姿与valid_for_motion=false边界不变。证据requested_eight_plan.json位于本轮忽略目录及Pi当前运行目录；未驱动车辆。


### 24占据却穿格的规划修正（2026-11-02 17:54后快照）

- 用户报告圆柱在24但动画穿过24。现场55圈占据24，原点距离判据的blocked_edges始终为空；已存计划也包含occupied24但第5段走3→6→7。根因是格子显示与按18cm点到线判据的规划不一致，并非仅旧快照未重算。
- 新增occupied_cell_edges：任一累计占据格与轴向路网边相交则封路，和原有点余量封路取并集；橙色边界待确认格也保守参与。点/格封路分别返回point_blocked_edges/cell_blocked_edges。当前启停格自身遮罩规则保留，不整格宣布free。
- 旧计划遇新阻断或扫描失效时动画在推进前冻结、隐藏箭头并禁用播放/重播，提示重算；恢复不自动续播。不每帧重算8段。阻断边改为红虚线/叉号/阻断字样，避免被误认作彩色路线。
- 35项相关Python测试及JS通过：两起始区20/24回波远离中心线仍封对应边、批次绕行、新占据使旧计划失效、过期和断连、动画冻结。真实bug24_before回放封边6/10，第5段改3→4→7。
- 核对PID27131命令后有序停止并确认串口释放；17:54:14启动PID42082，运行runtime_fix24_20261102_175414，旧3文件备份before/，6Hz最长3600秒约18:54到期。保持区2校准，未操作运动端。
- 新服务现场占据24、封边6/10，快照seq54的8/8计划总长13.60m，各段均不经过位于24的节点6；第5段为3→4→7。verified_plan.json保存在Pi当前运行目录。浏览器实际开始播放后注入仅浏览器的阻断响应，确认停止/禁用，撤销注入恢复实页；截图fixed24-routes.png在本轮忽略证据目录。仍为名义静止预览，未实车验证。

- 完成时运行状态又发生变化：先观测到占据20及另一已重算计划，随后start_zone变为1、pose为已保存区1值、mission_plan=None（配置变更清计划）。未覆盖此变更或按空场擅自重算；当前需以用户新摆放和区号为准。文件启动默认区2未由运行时切区自动改写。最终截图反映占据20时的阻断叉号，并非原24现场快照。
