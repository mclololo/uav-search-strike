"""

一次性任务生成脚本 —— 输入完整的自然语言任务描述，大模型一次性生成所有指令并发送

支持多模态图像输入（前视相机），用于九格多模态目标识别加分项。



用法:

    python3 mission_once.py "起飞，穿过框，识别二维码，再穿回来，去A区打红色气球，降落"

    python3 mission_once.py  (交互式输入)

"""

import sys

import os

import re

import gc

import numpy as np

from PIL import Image as PILImage

from transformers import AutoModel, AutoTokenizer

import torch

import rospy

from sensor_msgs.msg import Image

from std_msgs.msg import String





def ros_image_to_pil(msg: Image) -> PILImage.Image:

    """将 ROS Image 消息转换为 PIL RGB 图像"""

    enc = msg.encoding.lower()

    if enc in ('rgb8', 'bgr8'):

        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)

        if enc == 'bgr8':

            arr = arr[:, :, ::-1]

        return PILImage.fromarray(arr)

    elif enc == 'rgba8':

        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4)

        arr = arr[:, :, :3]

        return PILImage.fromarray(arr)

    elif enc == 'mono8':

        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)

        return PILImage.fromarray(arr).convert('RGB')

    else:

        raise ValueError(f"Unsupported image encoding: {msg.encoding}")





def load_model():

    """加载本地大模型和提示词"""

    script_dir = os.path.dirname(os.path.abspath(__file__))

    prompt_path = os.path.join(script_dir, "prompts", "mission_prompt.txt")

    model_dir = os.path.join(script_dir, "FM9G4B-V")



    if not os.path.exists(prompt_path):

        raise FileNotFoundError(f"Prompt file not found: {prompt_path}")

    if not os.path.isdir(model_dir):

        raise FileNotFoundError(f"Model directory not found: {model_dir}")



    with open(prompt_path, "r", encoding="utf-8") as f:

        knowledge_prompt = f.read()



    print("[1/3] 加载模型...", flush=True)

    model = AutoModel.from_pretrained(

        model_dir, trust_remote_code=True,

        attn_implementation='sdpa', torch_dtype=torch.bfloat16

    ).eval().cuda()

    tokenizer = AutoTokenizer.from_pretrained(model_dir, trust_remote_code=True)



    print("[3/3] 模型就绪。\n", flush=True)

    return model, tokenizer, knowledge_prompt





class MissionGenerator:

    def __init__(self, model, tokenizer, knowledge_prompt):

        self.model = model

        self.tokenizer = tokenizer

        self.knowledge_prompt = knowledge_prompt

        self.latest_image = None

        self.use_image_context = os.environ.get("MISSION_USE_IMAGE_CONTEXT", "0") == "1"

        self.image_sub = None

        if self.use_image_context:

            self.image_sub = rospy.Subscriber(

                '/rflysim/sensor1/img_rgb', Image, self.image_callback, queue_size=1

            )



    def image_callback(self, msg: Image):

        try:

            pil_img = ros_image_to_pil(msg)

            self.latest_image = pil_img.resize((224, 224), PILImage.BILINEAR)

        except Exception as e:

            rospy.logerr_throttle(5.0, f"[mission_once] Image conversion failed: {e}")



    def generate_commands(self, task_description: str):

        """向大模型发送任务描述，可附带当前图像，提取指令列表"""

        full_prompt = (

            self.knowledge_prompt.rstrip() + "\n\n"

            "用户: " + task_description + "\n"

            "回答:\n"

        )



        content = []

        if self.use_image_context and self.latest_image is not None:

            content.append(self.latest_image)

        content.append(full_prompt)



        msgs = [{"role": "user", "content": content}]



        result = self.model.chat(

            image=None, msgs=msgs, tokenizer=self.tokenizer,

            sampling=True, stream=True

        )

        generated = ""

        for t in result:

            print(t, flush=True, end="")

            generated += t

        print()



        gc.collect()

        torch.cuda.empty_cache()



        match = re.search(r"```(.*?)```", generated, re.DOTALL)

        if not match:

            return None

        code = match.group(1)

        if code.startswith("command"):

            code = code[len("command"):].strip()

        return code.strip()





def infer_commands_from_text(task_description: str):

    """根据用户原文做确定性解析，用来校正大模型漏项。"""

    text = task_description.lower().replace(" ", "")

    def has_any(words):

        return any(w in text for w in words)

    def has_negative_action(words):

        neg_words = ("不用", "不要", "无需", "不必", "不需要", "不")

        for word in words:

            idx = text.find(word)

            if idx < 0:

                continue

            window = text[max(0, idx - 4):idx]

            if any(neg in window for neg in neg_words):

                return True

        return False

    commands = []

    if has_any(("起飞", "升空", "takeoff")) and not has_negative_action(("起飞", "升空", "takeoff")):

        commands.append("takeoff")

    if has_any(("穿框", "穿过框", "过框", "穿越框", "框")) and not has_negative_action(("穿框", "穿过框", "过框", "穿越框", "框")):

        commands.append("cross_frame")

    if has_any(("二维码", "识别二维码", "扫二维码", "qr", "aruco")) and not has_negative_action(("二维码", "qr", "aruco")):

        commands.append("detect_aruco")

    if has_any(("反向穿框", "再穿框", "穿回来", "穿回框", "返回穿框", "再穿回来")) and not has_negative_action(("反向穿框", "再穿框", "穿回来", "穿回框", "返回穿框", "再穿回来")):

        commands.append("return_cross_frame")

    balloon_words = ("气球", "打气球", "打击", "刺", "扎", "击")

    if has_any(balloon_words) and not has_negative_action(balloon_words):

        if has_any(("蓝色", "蓝球", "blue")):

            color = "blue"

        elif has_any(("紫色", "紫球", "purple")):

            color = "purple"

        else:

            color = "red"

        commands.append(f"hit_balloon:{color}")

    if has_any(("降落", "land", "回到起点", "返回起点", "回原点", "返回原点", "返回")) and not has_negative_action(("降落", "land")):

        commands.append("land")

    return commands


def normalize_commands(task_description: str, model_commands: str):

    """优先使用用户原文推导出的完整序列，防止模型只输出部分动作。"""

    inferred = infer_commands_from_text(task_description)

    model_lines = []

    if model_commands:

        for line in model_commands.splitlines():

            cmd = line.strip()

            if cmd and not cmd.startswith("#"):

                model_lines.append(cmd)

    if not inferred:

        return "\n".join(model_lines)

    if model_lines != inferred:

        print("\n[校正] 模型输出与用户原文不一致，已按用户原文修正指令序列。", flush=True)

    return "\n".join(inferred)


def publish_mission_command(pub, commands: str, timeout: float = 10.0) -> bool:

    """可靠发送任务指令，等待 mission 节点订阅后重复发布，避免一次性消息丢失。"""

    deadline = rospy.Time.now() + rospy.Duration(timeout)

    while pub.get_num_connections() == 0 and not rospy.is_shutdown() and rospy.Time.now() < deadline:

        rospy.logwarn_throttle(2.0, "等待 /llm_mission_node 订阅 /mission_command ...")

        rospy.sleep(0.1)



    if pub.get_num_connections() == 0:

        rospy.logerr("/mission_command 当前没有订阅者，指令未发送。请确认 roslaunch mission_pkg llm_mission.launch 已启动。")

        return False



    msg = String(data=commands)

    for _ in range(5):

        pub.publish(msg)

        rospy.sleep(0.2)



    rospy.loginfo("指令已发送到 /mission_command")

    return True


def main():

    rospy.init_node("mission_once", anonymous=True)

    pub = rospy.Publisher("/mission_command", String, queue_size=10, latch=True)

    rospy.sleep(1.0)



    print("[0/3] 初始化 ROS 节点...", flush=True)

    model, tokenizer, knowledge_prompt = load_model()

    generator = MissionGenerator(model, tokenizer, knowledge_prompt)



    if len(sys.argv) > 1:

        task = " ".join(sys.argv[1:])

        print(f"\n[任务] {task}")

        print("=" * 60)

        try:

            commands = generator.generate_commands(task)

            if commands:

                commands = normalize_commands(task, commands)

                print(f"\n[生成的指令序列]\n{commands}\n")

                publish_mission_command(pub, commands)

            else:

                print("\n[错误] 大模型未返回有效指令。")

        except RuntimeError as e:

            print(f"\n[错误] 推理失败: {e}")

            torch.cuda.empty_cache()



    while not rospy.is_shutdown():

        print("\n" + "=" * 60)

        print("请输入任务描述（输入 quit 退出）：")

        try:

            task = input("> ").strip()

        except (EOFError, KeyboardInterrupt):

            print("\n退出。")

            break

        if not task:

            continue

        if task.lower() in ("quit", "exit", "q"):

            print("退出。")

            break



        print(f"\n[任务] {task}")

        print("=" * 60)



        try:

            commands = generator.generate_commands(task)

        except RuntimeError as e:

            print(f"\n[错误] 推理失败: {e}")

            torch.cuda.empty_cache()

            continue

        if not commands:

            print("\n[错误] 大模型未返回有效指令。")

            continue



        commands = normalize_commands(task, commands)

        print(f"\n[生成的指令序列]\n{commands}\n")

        publish_mission_command(pub, commands)





if __name__ == "__main__":

    main()