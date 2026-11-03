# 提前就绪、实体按键启动

`tools/chassis_button_service.py` 作为树莓派user systemd服务提前运行，独占电控UART、复用已有雷达采集；没有采集服务时按本机配置启动。雷达持续运行，必须收到STM32正常STATUS和新鲜扫描后才进入READY。无需Mac参与扫描、规划、上传或运动控制。

READY后，短按并松开KEY1（区1）或KEY0（区2），自动执行：ACK选区→选区后新扫描窗口→八段规划→完整几何校验→绑定请求上传→START→八站闭环→返回起点。仍保留至少4秒新扫描窗口，不在发令前缓存一条旧路线直接发车。默认100rpm巡航、20rpm短连接，固定航向+90°；实际本机配置在Git忽略的config/chassis_button.local.json，两区使用已确认的不同中心坐标。

## 与规则的关系

全国命题原文《02-2027年第十届 智能+工程创新赛道竞赛命题与运行》第7页要求调试结束后待机发车，裁判统一发令计时，一键式启动且只有一次启动机会；第1页要求比赛中完全自主运行。提前启动软件后，裁判发令时由实体按键启动，不需要途中远程START或继续确认。代码可以在一次完成后回到READY供后续练习；这不代表比赛允许同一轮重复启动，正式运行次数仍须遵守本场规则和裁判指令。

本入口仍是**底盘测试**：各站2秒模拟作业，不进行真实扫码、抓放、任务显示和计数。它落实一键自主启动的结构，不等于完整比赛功能或比赛验收通过。正式接口electrical_link.run_after_key仍要求真实作业完成反馈，不能用测试驻留冒充成功。

## 正式比赛入口也使用相同实体键启动

`tools/competition_button_service.py` 与 `deploy/competition-button.service` 是正式入口，复用同一个雷达预热和UART待机。KEY1/KEY0绑定选区后直接接正式执行，不需要第二次按键或电脑发START。每轮真实作业全部完成并收到DONE后自动回到READY；下一次新按键重新选择启停区，并重新采集和规划完整路线。两个systemd服务互斥，不能同时调度同一底盘。

正式本机配置 `config/competition_button.local.json` 增加 `competition_backend="真实模块:工厂函数"`。工厂接收本机配置，返回实际硬件适配对象，须满足：

- `simulation=False`；`ready()` 仅在真实视觉/机构等依赖已就绪时返回True。
- `plan(state, selection, route_id, check_current)` 返回已实车标定和允许运动的正式路线（valid_for_motion=True），不得带test_mode；使用指定route_id/start_zone，保持按键绑定。state为按键后新雷达计划，计算期间调用check_current以发现取消/换区。
- `on_work(client, status)` 是非阻塞的真实作业调度/反馈：None表示未完成；True表示本次作业确已成功；False表示失败。应按route_id/point_index去重，不得用固定驻留、mock输出伪造成功。它使用同一个UART所有者，不再启动ROS或另一线程抢读串口。

正式执行调用ElectricalClient.run_selected_route，保留作业门、请求绑定、超时和故障停止；不会运行测试版run_closed_loop或2秒定时释放。启动前仍复核扫描/几何及选区。正式后端未配置、未就绪时不会进入正式READY或发车，不自动回退测试。该后端涉及真实扫码/抓放/显示/统计，当前尚未接通；这里只完成正式一键启动和硬件流程接入入口，不宣称正式整车可参赛。

## 连续按键运行

- READY前按下的旧请求会被确认接收但忽略，不会在准备好后突然发车。上电时按住按键仍须按固件要求先松开。
- 不设置持久化的启动锁。READY后的每次新按键都会选择启停区、进行新的扫描和规划；完成后回到READY，失败后记录错误并等待控制器回到空闲再READY。
- 服务重启会重新读取控制器tick作为基线，重启前积压的旧按键会被忽略，不会自动重放路线。控制器处于运动或故障状态时只等待，不接收启动键。
- 多次运行是软件能力；正式比赛每轮启动次数仍按比赛规则执行，练习时可重复按键验证。
- STATUS超时或MCU重启时回到WAIT_CONTROLLER；通信恢复后重新建立READY基线。串口设备重枚举导致进程异常时由systemd重启服务。
- 设备断开/数据过期不会用旧计划启动；运动异常由现有run_closed_loop尝试STOP。电控没有通信断开自动停车的看门狗，此封装不新增该能力。

待机状态与每次状态转换记录在 `output/chassis_button/status.json`；全部UART TX/RX_RAW记录在wire.jsonl，准备快照和结果在prepared_live.json/result.json。STOP不是正常拐点控制命令。

查看状态（无需运行命令来解锁）：

```bash
cat /home/cuiguo/competition_vision/output/chassis_button/status.json
journalctl --user -u chassis-button.service -n 20 --no-pager
```

看到READY后，等裁判发令再按实体键。上电自动准备依赖user service已enable且用户开启linger；机器上电/断电启动能力需另外做完整断电实测，不能由一次systemctl启动成功推定。
