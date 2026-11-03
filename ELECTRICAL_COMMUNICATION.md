# 树莓派与电控通讯对接

更新：2026-11-02。先读AGENTS.md、PROJECT_CONTEXT.md。**当前以用户最新提供的 `/Users/jzxzhou/Downloads/car_code1 2/` 为基准**，已核对README_start_area.md、route.c、route_keys.c和route_client.py。旧拟议nonce/boot_id/revision区号协议已删除，不能再向新版电控发旧格式0x15/0x16。

## 接线与按键

用户已选USB转TTL，波特率115200、8N1、无流控、3.3V逻辑电平：USB-TTL TX→STM32 PC11(RX)，USB-TTL RX←STM32 PC10(TX)，GND共地。不接RS232电平，不用TTL电源脚给整车供电。STM32 UART5仍专供IMU。

Pi现有CP2102 by-id、ttyUSB0为雷达，电控适配器须单独确认稳定by-id路径。不能因型号也是CP2102就拒绝所有适配器，也不能默认打开第一个ttyUSB。工具拒绝解析后等于已知雷达设备的端口；无该稳定路径时保守拒绝ttyUSB0。新适配器尚未出现前不猜端口。USB-TTL不需要改GPIO的ttyAMA0控制台。

| 按键 | 空闲/READY/完成后 | 行驶或WAIT_WORK中 |
| --- | --- | --- |
| KEY1（PA15） | 选择启停区1，新请求使旧路线作废 | 忽略 |
| KEY0（PC5） | 选择启停区2，新请求使旧路线作废 | 立即取消路线；同次按住不触发新的扫描 |

30ms消抖；长按一次请求；双键同时按不选区；上电按住须先松开。任意键按住均禁止START。树莓派遇匹配READY且START返回BAD_STATE时最多等10秒，期间允许松开键/IMU就绪；取消、错误路线号不重试。KEY0停车语义来自新版固件，树莓派未改按键。

## 线路协议

帧：`A5 5A | version:u8=1 | type:u8 | sequence:u16 | payload_length:u16 | payload | CRC16:u16`。均小端。CRC16/CCITT-FALSE覆盖version至payload尾：poly1021、initFFFF、无反射、xorout0，`123456789→29B1`。最大payload1304、完整帧1314；普通路线内容最大1300；64航点。半包间隔超过500ms丢弃。

| type | 含义 | payload |
| --- | --- | --- |
| 0x10 | 原普通UPLOAD | 20字节头＋N×20；按键选择后固件拒绝，防绕过请求 |
| 0x11 | START | route_id:u32 |
| 0x12 | STOP，取消非暂停 | route_id:u32，0表示当前 |
| 0x13 | WORK_DONE | route_id:u32, point_index:u16, success:u8, reserved:u8=0 |
| 0x14 | STATUS | 空 |
| **0x15** | **AREA_ACK** | **request_id:u32, area:u8, reserved:3字节=0** |
| **0x16** | **UPLOAD_FOR_AREA** | **request_id:u32＋原UPLOAD完整payload** |
| 0x30 | VISION | 20字节，见下 |
| 0x80 | ACK | 36字节，sequence与request_type共同匹配 |
| 0x81 | EVENT | 同样36字节，独立事件计数 |
| **0x82** | **START_AREA** | **request_id:u32, tick:u32, area:u8, key:u8, reserved:u16=0，共12字节** |

START_AREA由STM32主动发送。未确认时每500ms重发相同request_id、tick、area；树莓派去重、只启动一次规划。KEY1对应area1/key1，KEY0对应area2/key0。AREA_ACK得到普通ACK OK后才调用规划。规划期间继续接收状态/按键；新请求使旧结果丢弃，以最新区号重新规划。带旧request_id的0x16即使迟到也会被固件拒绝。

request_id由STM32按键生成，route_id由Pi递增生成，两者不能混用。固件换区时route_status.route_id清0，但内部highest_id仍保留；所以不能按STATUS的0自动把新route_id重置为1。每轮手工提高--route-id，或由正式业务持久管理单调递增编号。MCU重启request_id不掉电保持；本版无boot_id，发现请求倒退/同号身份变化会要求重建Client。不声称可以检测所有快速掉电重启；重启后停止旧流程、重启Pi接收程序并重新按键。

旧 `pi_vision/protocol.py` 的AA帧仍保留给历史视觉基线，不能与A5 5A混发。相同数字命令号不是相同协议。一个UART只有一个Client持有，预览/视觉/任务请求交给该所有者，不再开第二个程序抢读。

## 路线与八段

### 双方统一的坐标与车头（2026-11-02）

**已与用户确认电控统一采用照片同向坐标 `field_photo_ccw`：场地照片同向视图的左下角为原点，+X向右、+Y向上，毫米；车头朝+X为0°、逆时针为正。** 0/90/180/−90°对应+X/+Y/−X/−Y。两个启停区共用原点和坐标轴。区1在左下，区2在左上，二维码区在左侧中段。

区1车头沿通道朝二维码区为 `initial_pose.yaw_deg=90`、测试 `heading_deg=90`，已写入双区示例；区2实际摆车方向未确认，yaw/heading留null。两区实际车体参考点X/Y留空，不使用启停区名义中心或雷达中心冒充测量。

内部雷达/路网仍使用已校准的原地图 `field_nominal`，不重写扫描校准：

```text
电控X_mm = 2400 − 原地图X_mm
电控Y_mm = 2400 − 原地图Y_mm
电控yaw_deg = wrap(原地图yaw_deg + 180°)
```

`electrical_coordinates`提供整个位姿的正反变换。配置输入已经是照片同向的车体X/Y/yaw；编译器只转换原路网节点（米→毫米并旋转180°），不会再旋转配置中的+90°。生成路线标记`coordinate_frame=field_photo_ccw`、`source_coordinate_frame=field_nominal`。初始与目标航向均直接使用新坐标配置中的度数，禁止只改yaw而保留旧X/Y。通用编译器、底盘测试配置和串口编码器强制显式frame；旧配置/无frame路线拒绝编码/上传，不能只给旧数据补标签。网页是否勾选180°显示不会更改协议坐标。

底盘测试将新坐标路线反变换后，在原规划地图核对起点、禁区及障碍；状态回传和到点对比均保持照片同向坐标。`pose_snapshot.frame_id=field_photo_ccw`，不自动将离散回传用于移动雷达定位。区1朝+90°，往二维码接近点时Y增大、对应前进；有绕障或起点连接时仍可能先横移。

上传必须提供车体实际初始航向；IMU上电0°不是地图0°。START时电控将当前IMU原始角锚定至上传的地图初始航向，`imu_yaw_sign=-1`由电控维护，Pi不再取反yaw。每个目标先转到目标yaw，再将地图位移换为前后/左右，较大分量先执行。测试所有目标yaw与初始yaw一致时，车头保持原方向，路径转弯通过麦轮横移/倒退完成；不会随路线箭头转头。

| 车头yaw | 地图+X位移 | 地图+Y位移 |
| --- | --- | --- |
| 0° | 前进 | 左移 |
| 90° | 右移 | 前进 |
| 180°/−180° | 后退 | 右移 |
| −90° | 左移 | 后退 |

`electrical_coordinates.direction_review`直接解码实际UPLOAD二进制，记录初值、每点目标、地图位移、前进/左移分量及先转后走顺序。通用Client在上传时保留`last_upload_review`；底盘测试在START前打印并保存`direction_review`。它是理想到点的方向核对，不改变发送内容，不代替反馈；电控有限转向误差可能引起少量额外修正。`(0,0,0)→(500,0,90)`显示“逆时针90°→右移500mm”；若要先前进后转，需增加`(500,0,0)`中间点。

当前路线编译器从原地图路网米坐标乘1000一次，再计算2400−X/2400−Y；Python输入航向已是照片同向的度数，串口编码乘100为0.01°。不接收ROS位姿作为自动起点，也不直接复制雷达pose；未来接入必须指定源地图坐标变换、弧度转度以及雷达相对车体的平移和偏航外参，得到车体参考点位姿后才上传。仅减安装偏角不足以修正雷达中心与车体中心的位置差。

离线验证入口（Mac终端，不开硬件）：

```bash
python3 -m tools.electrical_direction_check '/Users/jzxzhou/Downloads/car_code1 2' --output output/electrical_direction_20261102/check.json
```

已用真实route/heading/motion/IMU源码和模拟UART/CAN/轮子核验4个朝向下的±X/±Y共16种位移及两种先转/后转例子，18项通过；保持IMU符号−1，并检查模拟轮子产生的前后/左右位移及动作顺序。此为软件方向验证，真实电机接线、轮序和IMU安装仍须实车短距核对。

UPLOAD头 `<IHHiii`：route_id:u32非0、count:u16(1…64)、reserved:u16=0、实际车体initial_x_mm/initial_y_mm/initial_yaw_cdeg:i32。

每点 `<iiiHHHH`：x_mm、y_mm、yaw_cdeg:i32，move_rpm、turn_rpm、work_id、flags:u16。X/Y为照片同向绝对场地毫米±100000；yaw单位0.01°，−18000…18000；轮转速1…120rpm。flags=0/work_id=0为通行点，flags=1/work_id非0为工作点。Python输入yaw_deg，编码时乘100；路网米坐标转换成mm一次。最大64点超出即拒绝，不截断、不自动分成多个任务。

8段：启停→扫码→原料→加工→暂存→原料→加工→暂存→启停。末点标签101…108为本项目阶段约定，不是颜色或位置编号；本地保存段号与点索引映射，固件仅回传零起始point_index。第二批暂存仍按同颜色第一批物料码垛，正式业务不会用第四组码当第二批暂存位置。

STM32是底盘/航点推进唯一所有者。Pi批次规划与工作接口不同时启动完整mission_ros模拟层去调度同一底盘。ARRIVED在固件进入WAIT_WORK/写work_id之前发送，不能靠ARRIVED推进作业；用匹配WAIT_WORK，ACK只是收包处理。

注意：flags=0的“通行点”只是**不等待作业**，不是不停站通过；当前固件仍会闭环停稳。测试编译器默认合并同速同航向的顺向直线中间点，不再插入额外接近点；八个工作点及必要拐点保留。默认作业点的move_rpm是距离闭环的速度上限，不是到点速度。旧分段接近方式仅由stop_strategy=split_approach显式启用。

新测试模式见[CHASSIS_ROUTE_TEST.md](CHASSIS_ROUTE_TEST.md)：固定车头沿地图轴移动，末端低速接近，电控编码器/IMU闭环停稳，然后2秒模拟作业等待，8站后回实际起点。不读相机、不发VISION、不调用真实夹爪。通用正式接口没有定时完成，必须提供真实反馈。

原固件控制策略是先转到点目标yaw，再前/左分量先大后小执行；不是任意点间直线跟踪。电控已有剩余距离比例降速与RPM斜坡、编码器和IMU闭环；四轮反馈约±1rpm、指令近零、距离/角度在容差，持续250ms后动作DONE。当前每个航点均停稳，不是连续曲线巡航。

## 通用与测试入口

`pi_vision/electrical_link.py`提供：

- `wait_start_area(timeout)`：等待物理按键、ACK、返回request_id/tick/area/key；重复不返回。
- `plan_after_key(planner)`：先ACK再调用`planner(signal, check_current)`；planner在扫描/计算间隙调用check_current。换区抛AreaSuperseded，自动丢旧结果重入新区域；无上传/运动副作用。
- `upload(route, start_request_id=...)`：使用0x16并校验route.start_zone与已确认信号一致。正式路线需valid_for_motion=true；这不是物理验收的替代。当前预览不能手改true解锁。
- `run_after_key(planner, on_work=...)`：正式业务入口，按键→规划→绑定上传→等待键释放START→STATUS→真实工作反馈。`on_work(client,status)`必须快速返回：None=尚未完成、True=实际完成、False=失败；内部以路线号/点索引管理只启动一次实际任务。缺callback保持等待，到执行超时取消；绝不自动模拟成功。
- `tools/electrical_route_test.py --wait-key`：纯底盘测试入口；同一Client接收区号、选配置、规划和运行。只有本测试明确允许停留后模拟WORK_DONE。

正式业务接口已具备按键联动，但完整机械动作/视觉毫米标定仍没有完成，不能宣称整车业务已接通。`Route_OnWork`在提供固件里为空；将来加入机械动作后，底盘测试需禁用该钩子的机构动作。

## 状态与位姿

ACK/EVENT36字节 `<BBBBIHHHBBIIiihH`：request_type/code/state/phase:u8；route_id:u32；point_index/count/work_id:u16；chassis_fault/fault_address:u8；tick_ms/route_fault:u32；estimated_map_x/y:i32(mm)；yaw:i16(0.01°)；reserved:u16=0。

state：0EMPTY、1READY、2RUNNING、3WAIT_WORK、4DONE、5STOPPED、6FAULT。phase：0规划、1转向、2前后、3左右、4等待。
ACK结果：0OK、1BAD_TYPE、2BAD_LENGTH、3BAD_CRC保留、4BAD_VALUE、5BUSY、6BAD_ID、7BAD_STATE、8CONFLICT。
事件：1READY、2STARTED、3ARRIVED、4WORK_REQUIRED、5DONE、6STOPPED、7FAULT、8POINT_DONE。
路线故障：0无、1底盘、2启动/配置失败、3工作失败、4工作等待超时、5外部停止、6IMU掉线。

STATUS可补漏EVENT；单次匹配STATUS回包记录Pi本地单调接收时间，默认0.5秒过期清坐标。X/Y只在底盘子动作完成后累计，RUNNING中yaw跟随IMU；回包新鲜不等于XY测量新鲜。READY是上传初值，急停中途未完成位移可能未累计。输出continuous=false、valid_for_motion=false；不能用它给移动雷达做连续投影。回传采用照片同向场地图，+X为0°、逆时针为正；雷达外参和车体参考点仍待确认。

VISION `<IHH iihH`：route_id/point_index/work_id/dx_mm/dy_mm/dyaw_cdeg/confidence。偏移前方+X、左方+Y、逆时针正；confidence0…1000，0表示不可靠。只接受匹配WAIT_WORK；像素不能直接填mm。V1无颜色、任务码、测量期限字段，完整抓放协议仍需补齐。

## 停止与重连

同号同内容UPLOAD幂等、同号不同内容CONFLICT；活动中拒绝新路线。START活动态重试不重复启动；DONE/STOPPED/FAULT不能同号重跑。WORK_DONE仅最近一次支持重复确认。

异常/超时不能假装DONE或停稳。底盘测试异常尝试STOP再STATUS，记录确认或未确认；正式接口异常尽力STOP并抛原失败。MCU原固件**通信断开不自动停车**，STOP断线可能无法送达；KEY0本地取消仍有效。无链路看门狗、无动态障碍避停；当前流程仅用开局静止障碍快照，运动后不把旧静止位姿投影当新障碍。

## 无动作联调（部署后）

**树莓派终端**，先确认USB-TTL稳定路径，再用同一程序观察两键：

```bash
cd /home/cuiguo/competition_vision
python3 -m tools.electrical_comm --port /dev/serial/by-id/实际电控适配器 watch-areas --seconds 120
```

先运行再短按松开。预期KEY1输出area1、KEY0输出area2及递增request_id。该入口只确认信号，不规划、不上传、不START；输出本身不能算真实运动验收。一次只开一个串口程序。`status`仅查状态，`wait-area`只接一条新选区。

## 验证

当前Python覆盖新旧帧分流、两区映射、重发去重、换区中断规划、绑定上传、1314字节上限、重启拒绝、正式无反馈不放行。新版C主机测试20项通过，含route_keys消抖/双键/长按/活动KEY0停车和64点带请求上传。完整真实route/heading/motion/IMU加模拟UART/CAN/轮子，按键请求路径两区均完成8站回程；未接真实UART或电机。最终部署和测试数见PROJECT_CONTEXT.md。

部署已完成：26文件已同步Pi `/home/cuiguo/competition_vision`并校验SHA256；Pi345 passed/1 skipped、Mac346 passed/172 subtests，备份`backups/electrical_keys_20261102_28vEA6`。未启动运动或改系统UART。

## 2026-11-03 当前固件执行阶段扩展

实机CRC有效、匹配序号的36字节ACK已出现phase=5（RUNNING与STOPPED）。phase作为诊断执行阶段u8原样保留，不能再按旧0..4范围丢弃，否则会误报超时并触发STOP。route state、保留位、点数索引及CRC检查仍严格；WAIT_WORK作业释放仍检查state3/phase4，未知阶段不作为作业成功。此兼容修改没有定义phase5的控制语义，完整含义待最新版固件源码确认。
