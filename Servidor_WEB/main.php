<?php

if(isset($_POST['email']) && !empty($_POST['email']))
{
	$nome = addslashes($_POST['name']);
	$email = addslashes($_POST['email']);
	$mensagem = addslashes($_POST['message']);

	$to = "renan_klx@hotmail.com";
	$subject = "FORMULARIO TESTE - Contato";
	$body = "Nome: ".$nome. "\r\n".
			"Email: ".$email. "\r\n".
			"Mensagem: ".$message;

	$header = "From: renan@localhost.com"."\r\n".
				"Reply-To: ".$email."\r\n".
				"X=Mailer:PHP/".phpversion();

	if(mail($to,$subject,$body,$header))
	{
		echo("Email enviado com sucesso! \r\n");
	}
	else
	{
		echo("O e-mail não pode ser enviado \r\n");
	}
}

function getId()
{
	$path = getcwd()."/Dados/";
	$dataFile = $path."data.txt";
	$dataText = "0";
	try
	{
		$myfile = fopen($dataFile, "r");
		echo ("OK <br>");
		$dataText = fread($myfile,filesize($dataFile));
		echo ($dataText);
		echo("<br>lido");
	}
	catch (Exception $e) {
	    echo 'Exceção capturada: ',  $e->getMessage(), "\n";
	}
	$numero = 1 + (int)$dataText;
	$myfile = fopen($dataFile, "w") or die("Unable to open file!");
	$txt = $numero."\n";
	fwrite($myfile, $txt);
	fclose($myfile);

	return ($numero-1);
}

function createTask()
{
	$id = strval(getId());
	$tarefas = $_POST['name'];
	echo("<br>TAREFAS#");
	echo("<br>@".$tarefas."$");
	$path = getcwd()."/Dados/Tasks/";
	$dataFile = $path.$id."_task.txt";
	$myfile = fopen($dataFile, "w") or die("Unable to open file!");
	fwrite($myfile, $tarefas);
	fclose($myfile);
}

createTask();
?>
