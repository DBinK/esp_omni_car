min_val = 200
max_val = 1000

rate = -50

def map_value(value, original_block, target_block):
    """将给定的值映射到给定的目标范围。"""
    original_min, original_max = original_block
    target_min, target_max     = target_block
    # 计算映射后的值
    mapped_value = target_max + (value - original_min) * (target_min - target_max) / (original_max - original_min)

    return mapped_value

pwm_val = map_value(-rate, (0, 100), (min_val, max_val))

print(pwm_val)