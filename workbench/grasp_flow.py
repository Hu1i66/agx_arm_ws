"""Map existing T5 progress text to stable workbench flow stages."""


def grasp_stage_from_text(text):
    text = str(text or '')
    if '回合顺利完成' in text:
        return 'complete', '本轮完成'
    if '执行失败' in text or '抓取全部失败' in text or '错误' in text:
        return 'error', '本轮异常'
    if '闭合夹爪' in text:
        return 'grasp', '夹爪闭合'
    if '抬起脱离' in text or '抬起→' in text:
        return 'lift', '抬升搬运'
    if '松开夹爪' in text:
        return 'release', '释放物体'
    if '放置途经点' in text or '移动到放置点' in text or '移动到放置' in text:
        return 'place', '前往放置点'
    if '释放后返回' in text or '回到待机位' in text or '回观察位' in text:
        return 'return', '返回参考位'
    if '下降抓取' in text:
        return 'descend', '下降对齐'
    if '抓取位上方过渡点' in text or '预抓取' in text or '两阶段验证位' in text:
        return 'approach', '接近目标'
    if '收到客户端指令' in text or '次分拣' in text:
        return 'target', '目标已锁定'
    return None
