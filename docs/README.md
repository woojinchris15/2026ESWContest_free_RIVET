# CUBIC Documentation

`docs` 디렉터리는 CUBIC C1의 **설계, 개발 과정, 하드웨어 구성, 소프트웨어 구조, 테스트 및 3D 모델 관련 문서**를 정리한 공간이다.

상세 내용은 각 하위 디렉터리의 문서에서 확인할 수 있다.

---

## 📚 Documentation Index

| 구분                                                | 내용                                              |
| ------------------------------------------------- | ----------------------------------------------- |
| [`01_Project Overview`](./01_Project%20Overview/) | 개발 배경 · 목표 · 시스템 구성 · 역할 및 일정                   |
| [`02_Architecture`](./02_Architecture/)           | Mechanical · Electrical · Software Architecture |
| [`03_Datasheet`](./03_Datasheet/)                 | 주요 하드웨어 및 사용 부품                                 |
| [`04_Test`](./04_Test/)                           | 문제 해결 · Troubleshooting · 검증                    |
| [`05_Pic`](./05_Pic/)                             | 실물 및 프로젝트 이미지                                   |
| [`06_3Dmodel`](./06_3Dmodel/)                     | 전체 조립 및 개별 부품 STEP 모델                           |

---

## 01. Project Overview

CUBIC 프로젝트의 개발 배경, 목표, 시스템 구성 및 개발 일정을 정리한다.

→ [`01_Project Overview`](./01_Project%20Overview/)

---

## 02. Architecture

CUBIC의 전체 시스템 설계를 다음 세 영역으로 구분하여 정리한다.

* [`Mechanical Integration`](./02_Architecture/01_Mechanical%20Integration/)
* [`Electrical`](./02_Architecture/02_Electrical/)
* [`Software`](./02_Architecture/03_Software/)

---

## 03. Datasheet

CUBIC C1에 사용된 주요 하드웨어와 부품을 정리한다.

→ [`03_Datasheet`](./03_Datasheet/)

---

## 04. Test & Troubleshooting

개발 과정에서 발생한 주요 문제와 원인 분석, 해결 및 검증 과정을 정리한다.

→ [`04_Test`](./04_Test/)

---

## 05. Pictures

CUBIC의 실물 사진과 프로젝트 이미지를 포함한다.

→ [`05_Pic`](./05_Pic/)

---

## 06. 3D Model

전체 조립 모델, 개별 설계 부품 및 렌더 이미지를 포함한다.

→ [`06_3Dmodel`](./06_3Dmodel/)

---

## 🔗 관련 소스

| 구분                 | 경로                             |
| ------------------ | ------------------------------ |
| ROS 2 상위 제어        | [`../cubic_ws/`](../cubic_ws/) |
| MCU / PCU Firmware | [`../firmware/`](../firmware/) |
| systemd Services   | [`../systemd/`](../systemd/)   |

---

> 상세 설계와 개발 과정은 `docs`에서 설명하고, 실제 실행 및 빌드 방법은 각 소스 디렉터리의 README에서 설명한다.
