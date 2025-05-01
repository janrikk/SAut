# Tutorial para Configurar o X Server no WSL2 com VcXsrv

## 1. Instalar o VcXsrv

Baixe e instale o VcXsrv através do seguinte link:

👉 [https://sourceforge.net/projects/vcxsrv/](https://sourceforge.net/projects/vcxsrv/)

---

## 2. Configurar o VcXsrv

Após instalar o VcXsrv, siga os passos abaixo ao executá-lo:

1. Selecione **Multiple windows**
2. Em **Display number**, deixe como **0**
3. Clique em **Next**
4. Em "Client startup", selecione **Start no client**
5. Clique em **Next**
6. Em "Extra settings":
   - Mantenha marcada a opção **Clipboard**
   - **Desmarque** a opção do meio (**Primary Selection**)
   - Marque a opção **Disable access control**
7. Finalize clicando em **Finish**

---

## 3. Configurar o WSL2

Execute os seguintes comandos no terminal do WSL2 para configurar as variáveis de ambiente:

```bash
export GAZEBO_IP=127.0.0.1
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
export LIBGL_ALWAYS_INDIRECT=0
